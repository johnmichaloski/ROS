
#include "nist_fanuc.h"

#include <boost/format.hpp>
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>

#include "MotionControl.h"
#include "Globals.h"
#include "Controller.h"
#include "Kinematics.h"
#include "Communication.h"
#include "Setup.h"

#include <ros/package.h>
#include <ros/console.h>

#include "RvizMarker.h"
#include "NIST/TrajModel.h"
#include "NIST/BLogging.h"
#include "Debug.h"
using namespace KinematicChain;

// /opt/ros/indigo/include/moveit/robot_state/robot_state.h
// /opt/ros/indigo/include/moveit/move_group_interface/move_group.h
extern void SetupSceneObject();
extern void InitSceneObject();

int main(int argc, char** argv) {

    // Current robot joints declaration
    sensor_msgs::JointState cjoints;

    try {
        // Find path of executable
        std::string path(argv[0]);
        Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
        Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;
        setenv("ROS_ROOT", "/opt/ros/indigo/share/ros", true);
        setenv("ROS_PACKAGE_PATH", "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes:"
                "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes_core:"
                "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes_trajectory:"
                "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes_moveit:"
                "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes_planner:"
                "/usr/local/michalos/nistfanuc_ws/src/descartes/descartes_utilities:"
                "/usr/local/michalos/nistfanuc_ws/src/fanuc_lrmate200id_support:"
                "/usr/local/michalos/nistfanuc_ws/src/nist_fanuc:"
                "/usr/local/michalos/nistfanuc_ws/src/nistcrcl:"
                "/opt/ros/indigo/share:/opt/ros/indigo/stacks", true);
        setenv("ROS_MASTER_URI", "http://localhost:11311", true);
        setenv("ROS_DISTRO", "indigo", true);
        setenv("ROS_ETC_DIR", "/opt/ros/indigo/etc/ros", true);
        setenv("PYTHONPATH", "/usr/local/michalos/nistfanuc_ws/devel/lib/python2.7/dist-packages:"
                "/usr/local/michalos/nistcrcl_ws/devel/lib/python2.7/dist-packages:"
                "/opt/ros/indigo/lib/python2.7/dist-packages:"
                "/home/isd/michalos/el-robotics-core/nist_kitting/src", true);

        setenv("PKG_CONFIG_PATH", "/usr/local/michalos/nistfanuc_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/usr/local/michalos/nistcrcl_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/x86_64-linux-gnu/pkgconfig:/usr/local/michalos/nistfanuc_ws/devel/lib/pkgconfig:/usr/local/michalos/nistcrcl_ws/devel/lib/pkgconfig:/opt/ros/indigo/lib/pkgconfig:/usr/lib/pkgconfig:/usr/local/lib/pkgconfig", true);
        setenv("PATH", "/usr/local/michalos/nistfanuc_ws/devel/bin:/usr/local/michalos/nistcrcl_ws/devel/bin:/opt/ros/indigo/bin:/usr/local/jdk1.8.0_60/bin:/bin:/usr/bin:/usr/local/bin:/sbin:/usr/sbin:/usr/local/sbin:/usr/X11R6/bin:/usr/local/ulapi/bin:/usr/local/gomotion/bin:/home/isd/michalos/bin", true);

        // This sets up some application name/value pairs: user, hostname
        SetupAppEnvironment();

        //SetupRosEnvironment - needs to go before ROS!
        //SetupRosEnvironment(""); // FAILS hard coded env above

        // Initialize ROS
        ros::init(argc, argv, "nist_fanuc");
        ros::NodeHandle nh;
        ros::Rate r(50); // 10 times a second - 10Hz


        // Accessing Private Parameters
        // ros::param::get("~private_name", param); 
        boostlogfile = nh.param<std::string>("logfile", "/home/isd/michalos/Documents/example.log");
        boostloglevel = (boost::log::v2_mt_posix::trivial::severity_level) nh.param<int>("loglevel", 0); // 0 = debug
        Cnc.bCvsPoseLogging() = nh.param<int>("csvlogging", 0);
        Cnc.CvsPoseLoggingFile() = nh.param<std::string>("csvlogfile", "/home/isd/michalos/Documents/nistcrcl.csv");

        // THIS DOESN'T WORK
#if 0
        int rosloglevel = nh.param<int>("~rosloglevel", 0); // 0 = debug

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, (ros::console::levels::Level) rosloglevel)) {
            ros::console::notifyLoggerLevelsChanged();
        }
#endif  
        LOG_DEBUG << ExecuteShellCommand("env|sort\n");


        // Controller shared objects dependent on ROS - many with abstract interface definition
        boost::shared_ptr<CJointReader>jointReader;
        boost::shared_ptr<CJointWriter>jointWriter;
        boost::shared_ptr<IKinematics> kin;
        boost::shared_ptr<MoveitPlanning> moveitPlanner;
        boost::shared_ptr<CRvizMarker> pRvizMarker;
        boost::shared_ptr<CLinkReader> pLinkReader;




        //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
        ros::AsyncSpinner spinner(1);
        spinner.start();


        // This is useful for rosbag i suppose
        //        std::string run_id;
        //        nh.getParam("run_id", run_id);
        //        std::cout << run_id.c_str() << std::endl;

        // ROS config - parameter list - save for comparison later
        std::string params = ReadRosParams(nh);
        Globals.WriteFile(Globals.ExeDirectory + "rosconfig.txt", params);
        path = ros::package::getPath("nist_fanuc");
        Globals._appproperties["nist_fanuc"] = path;
#if 0
        // This sets up the `env` so that ROS can run - has too many hardwired dependencies
        SetupRosEnvironment(path);
#endif
        // Controller instantiatio of shared objects  - dependent on ROS
        jointReader = boost::shared_ptr<CJointReader>(new CJointReader(nh));
        jointWriter = boost::shared_ptr<CJointWriter>(new CJointWriter(nh));
        pRvizMarker = boost::shared_ptr<CRvizMarker>(new CRvizMarker(nh));
        pRvizMarker->Init();
        pLinkReader = boost::shared_ptr<CLinkReader>(new CLinkReader(nh));

//#define ARMKIN
#ifdef ARMKIN
        kin = boost::shared_ptr<IKinematics>(new ArmKinematics());
        // Initializatin of Controller instantiatio of shared objects  
        kin->Init(std::string("manipulator"), std::string("tool0"));
        kin->Init(nh);
        RCS::Cnc.Kinematics() = kin;
#endif
#define FASTKIN    
#ifdef FASTKIN
        kin = boost::shared_ptr<IKinematics>(new FastKinematics());
        // Initializatin of Controller instantiatio of shared objects  
        kin->Init(std::string("manipulator"), std::string("tool0"));
        kin->Init(nh);
        RCS::Cnc.Kinematics() = kin;
#endif

//#define MOVEITKIN      
#ifdef MOVEITKIN
        kin = boost::shared_ptr<IKinematics>(new MoveitKinematics(nh));
        // Initializatin of Controller instantiatio of shared objects  
        kin->Init(std::string("manipulator"), std::string("tool0"));
        RCS::Cnc.Kinematics() = kin;
#endif
#ifdef MOVEITPLANNER
        moveitPlanner = boost::shared_ptr<MoveitPlanning>(new MoveitPlanning(nh));
        RCS::Cnc.MoveitPlanner() = moveitPlanner;
#endif

        // Initialize Controller...
        RCS::Cnc.Setup(nh);
        //RCS::Controller._NumJoints = 6; // hard code even thought chainrobotmodel will work
        RCS::Cnc.status.Init();
        RCS::Cnc.CycleTime() = DEFAULT_LOOP_CYCLE;

        #if 1
        //http://www.radmangames.com/programming/how-to-use-boost-bind
        RCS::Pose Base(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));

        RCS::Pose Robot(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        RCS::Pose Gripper(Quaternion(0, 0, 0, 1), Vector3(.120, 0, 0));
        RCS::Pose Table(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.25, -.45, 0.35));
        // Works
        // RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, 0.695));
       // RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.465, 0, 0.695));
        //RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, .335));

        KinematicChain::MotionEquation chain;
        chain.make_equation("Test", kin,
                KinematicChain::MotionEquation::BASE, 
                KinematicChain::MotionEquation::ROBOT, 
                KinematicChain::MotionEquation::TOOL, 
                KinematicChain::MotionEquation::EQUALS,
                KinematicChain::MotionEquation::TABLE, 
                KinematicChain::MotionEquation::GOAL,
                KinematicChain::MotionEquation::DONE
                );
        //chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
        chain.SetPose( KinematicChain::MotionEquation::GOAL, GoalPose);
        chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
        chain.SetPose( KinematicChain::MotionEquation::TOOL, Gripper);
       std::vector<double> joints = chain. Solve();
#endif
       

        //        RCS::Controller.TrajectoryWriter() = trajWriter;
        RCS::Cnc.JointWriter() = jointWriter;
        RCS::Cnc.RvizMarker() = pRvizMarker;
        RCS::Cnc.EEPoseReader() = pLinkReader;

        RCS::Cnc.eCartesianMotionPlanner = RCS::CController::BASIC;
        RCS::Cnc.eJointMotionPlanner = RCS::CController::BASIC;
        // RCS::CController::MOVEIT;  NOPLANNER=0, MOVEIT, DESCARTES, BASIC, WAYPOINT, GOMOTION         

        //#define INITJOINTCONTROLLER
#ifdef INITJOINTCONTROLLER
        // Read latest values, dont start until they are read
        jointReader->Start();
        cjoints = jointReader->GetCurrentReadings();
        while (cjoints.position.size() == 0) {
            //ros::spinOnce();
            cjoints = jointReader->GetCurrentReadings();
            Globals.Sleep(100);
            //r.sleep();
            std::cout << "." << std::flush;
        }
        LOG_DEBUG << "\nCurrent joints=" << VectorDump<double> (cjoints.position).c_str();

        // Store current joint values
        //RosKinematics kin;
        RCS::Cnc.status.currentjoints = cjoints;
        LOG_DEBUG << "Current=" << VectorDump<double> (RCS::Cnc.status.currentjoints.position).c_str();
        RCS::Controller.status.currentpose = kin->FK(RCS::Cnc.status.currentjoints.position);
        LOG_DEBUG << DumpPose(RCS::Controller.status.currentpose).c_str();
        //        RCS::Cnc.CrclDelegate()->crclwm.Update(RCS::Cnc.status.currentpose);
        //        RCS::Cnc.CrclDelegate()->crclwm.Update(RCS::Cnc.status.currentjoints);

        LOG_DEBUG << "Starting current joints=" << DumpJoints(cjoints).c_str();
        LOG_DEBUG << "Starting current pose=" << DumpPose(RCS::Controller.status.currentpose).c_str();

#endif

#if DESCARTES
        boost::shared_ptr<CTrajectory> traj = boost::shared_ptr<CTrajectory> (new CTrajectory());
        // Descarte trajectory writer - uses action lib
        //boost::shared_ptr<CTrajectoryWriter> trajWriter = boost::shared_ptr<CTrajectoryWriter>(new CTrajectoryWriter(traj));

        // INitialize this if you are using Descartes
        // Create a robot model and initialize trajectory with it
        const std::string robot_description = "robot_description";
        const std::string group_name = "manipulator"; // name of the kinematic group
        const std::string world_frame = "/base_link"; // Name of frame in which you are expressing poses.
        const std::string tcp_frame = "tool0"; // tool center point frame
        std::vector<std::string> names;
        // This assumes the yaml file containgin ros param controller_joint_names is loaded
        nh.getParam("controller_joint_names", names);
        RCS::Controller.TrajectoryModel() = traj;
        RCS::Controller.TrajectoryModel()->Init(robot_description, group_name, world_frame, tcp_frame, names);
#endif

        //#define ROBOTSTATUS
#ifdef ROBOTSTATUS
        RCS::RobotStatus robotstatus;
        //       robotstatus.CrclDelegate() = crcl;
        robotstatus.JointReader() = jointReader;
        robotstatus.CycleTime() = DEFAULT_LOOP_CYCLE;
#ifdef MOVEITKIN
        robotstatus.Kinematics() = kin;
#endif
        //        robotstatus.Start(); // start the controller status thread
#endif
        InitSceneObject();
        SetupSceneObject();
        //       jointWriter->Start();
        RCS::Cnc._interpreter = boost::shared_ptr<RCSInterpreter>(new BangBangInterpreter());
        RCS::Cnc.Start(); // start the Controller Session thread

        spinner.stop();
        ros::spin();
        //        do {
        //            ros::spinOnce();
        //            r.sleep();
        //        } while(ros::ok());
        //        
        LOG_DEBUG << "Cntrl C pressed \n" << std::flush;

        // ^C pressed - stop all threads or will hang
        if (jointReader)
            jointReader->Stop(); // unsubscribe
        if (jointWriter)
            jointWriter->Stop(); // unpusblish
        RCS::Thread::StopAll(); // includes thread for Controller, robotstatus

    } catch (std::exception e) {
        LOG_FATAL << Globals.StrFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.what());
    } catch (...) {
        LOG_FATAL << "Abnormal exception end to  CRCL2Robot";
    }
    ros::shutdown();
    return 0;
}


