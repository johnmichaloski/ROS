

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

//#pragma message "Compiling " __FILE__ 

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
#include "RosSetup.h"
#include "RvizMarker.h"
#include "NIST/BLogging.h"
#include "Debug.h"
#include "Scene.h"
#include "Checkerboard.h"
#include "StackTrace.h"
#include "RCSInterpreter.h"
#include "Demo.h"
#include "Test.h"

#define MULTITHREADED
#define MOTOMAN
#define FANUC
// /opt/ros/indigo/include/moveit/robot_state/robot_state.h
// /opt/ros/indigo/include/moveit/move_group_interface/move_group.h

//Eigen::Affine3d fanucoffset00 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -0.5, 0.0);
//Eigen::Affine3d motomanoffset00 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 0.0);

tf::Pose fanucbaseoffset = RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf::Vector3(0.0, -0.5, 0.0));

tf::Pose motomanbaseoffset = RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf::Vector3(0.0, 0.5, 0.0));

//tf::Pose fanucGripper = RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),  tf::Vector3(0.140, 0.0, -0.017));
tf::Pose fanucGripper = RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),  tf::Vector3(0.017, 0.0,  0.140));

tf::Pose motoGripper = RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf::Vector3(-0.017, 0.0, 0.140));

int main(int argc, char** argv) {
    int bPublishPoint = 0;
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

#if 0
        /**
         * The five different ROS verbosity levels are, in order:
         * DEBUG ROS_DEBUG
         * INFO ROS_INFO
         * WARN
         * ERROR
         * FATAL  ROS_FATAL
         */

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
#endif
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


        //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
        ros::AsyncSpinner spinner(2);
        spinner.start();

        // ROS config - parameter list - save for comparison later
        std::string params = ReadRosParams(nh);
        Globals.WriteFile(Globals.ExeDirectory + "rosconfig.txt", params);
        path = ros::package::getPath("nist_fanuc");
        Globals._appproperties["nist_fanuc"] = path;

        // Controller shared objects dependent on ROS - many with abstract interface definition   

        // Initialize Controller...
#ifdef FANUC
        // ikfast kinematic solver
        RCS::Fnc->SetBaseOffset(fanucbaseoffset);
        RCS::Fnc->SetToolOffset(fanucGripper);     
        //RCS::Fnc->QBend() = tf::Quaternion(M_PI / 2.0, 0.0, 0.0);
        //RCS::Fnc->QBend() = tf::Quaternion(0.0, M_PI / 2.0, 0.0);
        RCS::Fnc->QBend() = tf::Quaternion(Deg2Rad(180.0), Deg2Rad(0.0), Deg2Rad(0.0));
        boost::shared_ptr<IKinematics> fkin;
        fkin = boost::shared_ptr<IKinematics>(new FanucLRMate200idFastKinematics());
        fkin->Init(std::string("manipulator"), std::string("fanuc_link_6"), std::string("world"));
        fkin->Init(nh);
        RCS::Fnc->Kinematics() = fkin;
        RCS::Fnc->CycleTime() = DEFAULT_LOOP_CYCLE;
        RCS::Fnc->NamedJointMove["Safe"] = ToVector<double>(6, 1.49, -0.17, -1.14, 0.11, -0.45, -1.67);
        RCS::Fnc->Setup(nh, "fanuc_");
        //FanucNearestJointsLookup fanuchints(RCS::Fnc, fkin);
        // hints fail, and I need to factor in robot base offset
        //fanuchints.SetRobotHints();  
        InlineRobotCommands fanucrobot(RCS::Fnc); // , fanuchints);
        RCS::Fnc->Kinematics() = fkin;
        RCS::Fnc->_interpreter = boost::shared_ptr<IRCSInterpreter>(new RCS::BangBangInterpreter(RCS::Fnc, fkin)); // , fanuchints));
#endif

#ifdef MOTOMAN
        RCS::Mnc->SetBaseOffset(motomanbaseoffset);
        RCS::Mnc->SetToolOffset(motoGripper);
        RCS::Mnc->QBend() = tf::Quaternion(Deg2Rad(-180.0), Deg2Rad(0.0), Deg2Rad(0.0));
        boost::shared_ptr<IKinematics> fastkin;
        fastkin = boost::shared_ptr<IKinematics>(new MotomanSia20dFastKinematics());
        // Initialization of Controller instantiation of shared objects  
        fastkin->Init(std::string("manipulator"), std::string("motoman_link_t"), std::string("world"));
        fastkin->Init(nh);
        //        LOG_DEBUG << "Motoman Joint Names=" << VectorDump<std::string>(fastkin->JointNames()).c_str();
        RCS::Mnc->Kinematics() = fastkin;
        RCS::Mnc->Setup(nh, "motoman_");
        RCS::Mnc->CycleTime() = DEFAULT_LOOP_CYCLE;

        RCS::Mnc->NamedJointMove["Safe"] = ToVector<double>(7, 1.30, -0.84, 0.08, 2.26, 2.96, -0.38, -1.28);

        //MotomanNearestJointsLookup motohints(RCS::Mnc, fastkin);
        //motohints.SetRobotHints();
        InlineRobotCommands motomanrobot(RCS::Mnc); // , motohints);
        RCS::Mnc->_interpreter = boost::shared_ptr<IRCSInterpreter>(new RCS::BangBangInterpreter(RCS::Mnc, fastkin)); // , motohints));
#endif

        InitScene();

#ifdef CHECKERS
        RvizCheckers rvizgame(nh);
        rvizgame.RvizSetup();
#endif

        DrawScene(); // Debug: LOG_DEBUG << ObjectDB::DumpDB();

#ifdef   MULTITHREADED
#ifdef FANUC   
                RCS::Fnc->Start(); // start the Controller Session thread
#endif
#ifdef MOTOMAN
                RCS::Mnc->Start(); // start the Controller Session thread
#endif
#endif

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
            if (player == Checkers::RED) {
                LOG_DEBUG << "Fanuc RED Move";
                rvizgame.PhysicalMove(fanucrobot, player, from.row, from.col, to);
            } else {
                LOG_DEBUG << "Motoman BLACK Move";
                rvizgame.PhysicalMove(motomanrobot, player, from.row, from.col, to);
            }
#ifndef   MULTITHREADED
            while (RCS::Mnc->IsBusy()) {
                RCS::Mnc->Action();
            }
#endif
            // Synchronize with rviz let PublishPoint pause execution
            if (rvizdemo.Clicked()) {
                RCS::Fnc->Suspend();
                RCS::Mnc->Suspend();
                while (1) {
                    ros::spinOnce();
                    ros::Duration(0.2).sleep();
                    if (rvizdemo.Clicked())
                        break;
                }
                RCS::Fnc->Resume();
                RCS::Mnc->Resume();
            }
            ros::spinOnce();
                    ros::spinOnce();
                    ros::Duration(0.2).sleep();
        }
#endif
        while (!rvizdemo.Ready()) {
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
        } while (ros::ok());
                spinner.stop();
                LOG_DEBUG << "Cntrl C pressed \n" << std::flush;

                // ^C pressed - stop all threads or will hang
                RCS::Thread::StopAll(); // includes thread for Controller, robotstatus

        } catch (std::exception e) {
        LOG_FATAL << Globals.StrFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.what());
    } catch (...) {
        LOG_FATAL << "Abnormal exception end to  CRCL2Robot";
    }
    ros::shutdown();
    return 0;
}


