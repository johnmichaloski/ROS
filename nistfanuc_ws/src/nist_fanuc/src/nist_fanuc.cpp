
#include "nist_fanuc.h"

#include <boost/format.hpp>
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>


#include <ros/package.h>
#include <ros/console.h>


#include "MotionControl.h"
#include "Globals.h"
#include "Controller.h"
#include "Kinematics.h"
#include "Communication.h"
#include "RosSetup.h"
#include "RvizMarker.h"
#include "NIST/BLogging.h"
#include "Debug.h"
#include "Scene.h"
#include "Checkerboard.h"
#include "StackTrace.h"
#include "fanuc_lrmate200id.h"
#include "RCSInterpreter.h"
#include "Demo.h"
#include "Test.h"

// /opt/ros/indigo/include/moveit/robot_state/robot_state.h
// /opt/ros/indigo/include/moveit/move_group_interface/move_group.h
extern RCS::Pose ComputeGripperOffset();
extern RCS::Pose AutoComputeGripperOffset(urdf::Model& robot_model, std::string prefix);


int main(int argc, char** argv) {
    // Current robot joints declaration
    sensor_msgs::JointState cjoints;
    int bPublishPoint=0;
    //signal(SIGSEGV, handler);   // install our handler
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

       // setenv("PKG_CONFIG_PATH", "/usr/local/michalos/nistfanuc_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/usr/local/michalos/nistcrcl_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/x86_64-linux-gnu/pkgconfig:/usr/local/michalos/nistfanuc_ws/devel/lib/pkgconfig:/usr/local/michalos/nistcrcl_ws/devel/lib/pkgconfig:/opt/ros/indigo/lib/pkgconfig:/usr/lib/pkgconfig:/usr/local/lib/pkgconfig", true);
        setenv("PATH", "/usr/local/michalos/nistfanuc_ws/devel/bin:/usr/local/michalos/nistcrcl_ws/devel/bin:/opt/ros/indigo/bin:/usr/local/jdk1.8.0_60/bin:/bin:/usr/bin:/usr/local/bin:/sbin:/usr/sbin:/usr/local/sbin:/usr/X11R6/bin:/usr/local/ulapi/bin:/usr/local/gomotion/bin:/home/isd/michalos/bin", true);

        // This sets up some application name/value pairs: user, hostname
        SetupAppEnvironment();

        //SetupRosEnvironment - needs to go before ROS!
        //SetupRosEnvironment(""); // FAILS hard coded env above

        // Initialize ROS
        ros::init(argc, argv, "nist_fanuc");
        ros::NodeHandle nh;
        ros::Rate r(50); // 10 times a second - 10Hz

        /**
         * The five different verbosity levels are, in order:
         * DEBUG ROS_DEBUG
         * INFO ROS_INFO
         * WARN
         * ERROR
         * FATAL  ROS_FATAL
         */
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    
        RvizDemo rvizdemo(nh);

        // Accessing Private Parameters
        // ros::param::get("~private_name", param); 
        boostlogfile = nh.param<std::string>("logfile", "/home/isd/michalos/Documents/example.log");
        boostloglevel = (boost::log::v2_mt_posix::trivial::severity_level) nh.param<int>("loglevel", 0); // 0 = debug
        RCS::Fnc->bCvsPoseLogging() = nh.param<int>("csvlogging", 0);
        RCS::Fnc->CvsPoseLoggingFile() = nh.param<std::string>("csvlogfile", "/home/isd/michalos/Documents/nistcrcl.csv");
        bPublishPoint = nh.param<int>("PublishPoint", 0);
        // THIS DOESN'T WORK
#if 0
        int rosloglevel = nh.param<int>("~rosloglevel", 0); // 0 = debug

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, (ros::console::levels::Level) rosloglevel)) {
            ros::console::notifyLoggerLevelsChanged();
        }
#endif  
//        LOG_DEBUG << ExecuteShellCommand("env|sort\n");


        // Controller shared objects dependent on ROS - many with abstract interface definition
        boost::shared_ptr<CJointReader>jointReader;
        boost::shared_ptr<CJointWriter>jointWriter;
        boost::shared_ptr<IKinematics> kin;
        boost::shared_ptr<CRvizMarker> pRvizMarker;
        boost::shared_ptr<CLinkReader> pLinkReader;

        //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
        ros::AsyncSpinner spinner(2);
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
        
        tf::Pose baseoffset =Conversion::Affine3d2RcsPose(fanucoffset00);

#define ARMKIN
#ifdef ARMKIN
 #ifdef FANUCPREFIX
        kin = boost::shared_ptr<IKinematics>(new ArmKinematics("fanuc_", baseoffset));
#else
        kin = boost::shared_ptr<IKinematics>(new ArmKinematics("", ));
#endif
        // Initialization of Controller instantiation of shared objects  
        // Fixme: there is no tool0?
        kin->Init(std::string("manipulator"), std::string("tool0"));
        kin->Init(nh);
        RCS::Fnc->Kinematics() = kin;
        ComputeGripperOffset();
#ifdef FANUCPREFIX
        AutoComputeGripperOffset(kin->armkin->robot_model, "fanuc_");
#else
        AutoComputeGripperOffset(kin->armkin->robot_model, "");
#endif

        TestFk(kin, "armkin");
        TestIk(kin, "armkin");

#endif
//#define FASTKIN    
#ifdef FASTKIN
        boost::shared_ptr<IKinematics> fastkin;
        fastkin = boost::shared_ptr<IKinematics>(new FastKinematics());
        // Initializatin of Controller instantiatio of shared objects  
        fastkin->Init(std::string("manipulator"), std::string("tool0"));
        fastkin->Init(nh);
        //RCS::Fnc->Kinematics() = fastkin;
        TestFk(fastkin, "fastkin");
        TestIk(fastkin, "fastkin");
#endif

//#define lrmate200KIN    
#ifdef lrmate200KIN
        boost::shared_ptr<IKinematics> lrmate200idkin;
        lrmate200idkin = boost::shared_ptr<IKinematics>(new FanucLrMate200idKinematics());
        lrmate200idkin->Init(std::string("manipulator"), std::string("tool0"));
        lrmate200idkin->Init(nh);
        TestFk(lrmate200idkin, "lrmate200idkin");
        TestIk(lrmate200idkin, "lrmate200idkin");  
#endif

        // Initialize Controller...
#ifdef FANUCPREFIX
        RCS::Fnc->Setup(nh, "fanuc_");
#else
        RCS::Fnc->Setup(nh, "");
#endif
        //RCS::Controller._NumJoints = 6; // hard code even thought chainrobotmodel will work
        RCS::Fnc->status.Init();
        RCS::Fnc->CycleTime() = DEFAULT_LOOP_CYCLE;

        //        RCS::Controller.TrajectoryWriter() = trajWriter;
        RCS::Fnc->JointWriter() = jointWriter;
        RCS::Fnc->RvizMarker() = pRvizMarker;
        RCS::Fnc->EEPoseReader() = pLinkReader;

        RCS::Fnc->eCartesianMotionPlanner = RCS::CController::BASIC;
        RCS::Fnc->eJointMotionPlanner = RCS::CController::BASIC;
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
        RCS::Fnc->status.currentjoints = cjoints;
        LOG_DEBUG << "Current=" << VectorDump<double> (RCS::Fnc->status.currentjoints.position).c_str();
        RCS::Controller.status.currentpose = kin->FK(RCS::Fnc->status.currentjoints.position);
        LOG_DEBUG << DumpPose(RCS::Controller.status.currentpose).c_str();
        //        RCS::Fnc->CrclDelegate()->crclwm.Update(RCS::Fnc->status.currentpose);
        //        RCS::Fnc->CrclDelegate()->crclwm.Update(RCS::Fnc->status.currentjoints);

        LOG_DEBUG << "Starting current joints=" << DumpJoints(cjoints).c_str();
        LOG_DEBUG << "Starting current pose=" << DumpPose(RCS::Controller.status.currentpose).c_str();

#endif
        FanucNearestJointsLookup fanuchints(Conversion::Affine3d2RcsPose(fanucoffset00), kin);
        fanuchints.SetRobotHints();
        InlineRobotCommands fanucrobot(RCS::Fnc, fanuchints);
        

        InitScene();
 #if 0       
        RvizCheckers rvizgame(nh);
        Checkers::BoardType outboard;
        std::stringstream str;
        str<< rvizgame.Game().TestBoard();

        LOG_DEBUG << str.str().c_str();
        rvizgame.Game().Deserialize(str, outboard);
         LOG_DEBUG << rvizgame.Game().printDisplayFancy(outboard).c_str();
#endif
#ifdef CHECKERS
        RvizCheckers rvizgame(nh);
        rvizgame.RvizSetup();
#endif
        DrawScene();
       // LOG_DEBUG << ObjectDB::DumpDB();
        
        RCS::Fnc->_interpreter = boost::shared_ptr<IRCSInterpreter>(new RCS::BangBangInterpreter( RCS::Fnc, kin));
        RCS::Fnc->Kinematics() = kin;
        RCS::Fnc->Start(); // start the Controller Session thread
        fanucrobot.AddGripperOffset();
        
#ifdef CHECKERS
        // Play checkers - only move markers, no robot interaction
        Checkers::Move from, to;
        int player;
        for (size_t i = 0; i < 40; i++) {
            if (bPublishPoint) {
                while (!rvizgame.Ready())
                    ros::spinOnce();
                rvizgame.Ready() = false;
            }
            if (rvizgame.CheckersMove(player, from, to))
                break;
            rvizgame.Game().printDisplayFancy(rvizgame.Game().Board());
            rvizgame.PhysicalMove(fanucrobot, player, from.row, from.col, to);
            ros::spinOnce();
            ros::spinOnce();
            ros::Duration(0.2).sleep();
        }
#endif
       while(!rvizdemo.Ready() ) {
           ros::Duration(1.0).sleep();
       }


        //spinner.stop();
        //      ros::spin();
        do {
            //for(size_t i=0; i< 10; i++)
             //   ros::spinOnce();
#ifdef BOLTDEMO
            if (RCS::Fnc->crclcmds.SizeMsgQueue() == 0) {
                ClearScene();
                NewScene();
                DrawScene();
                fanucrobot.TestRobotCommands();
            }
#endif           
            r.sleep();
        } while(ros::ok());
        spinner.stop();    
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


