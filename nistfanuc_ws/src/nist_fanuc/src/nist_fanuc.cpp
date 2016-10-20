

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
#include "Kinematics.h"
#include "Controller.h"
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
#include "Config.h"
#include "nist_fanuc/MotionException.h"
//Eigen::Affine3d fanucoffset00 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -0.5, 0.0);
//Eigen::Affine3d motomanoffset00 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 0.0);

int main(int argc, char** argv) {
    int bPublishPoint = 0;
    //signal(SIGSEGV, handler);   // install our handler
    try {
        // Find path of executable
        std::string path(argv[0]);
        Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
        Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;

        // This hard coding of env variables is required for debugging with netbeans ide
        // If ROS environment variables are not set it cannot find "stuff"
#ifdef DEBUG       
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
#endif

        // This sets up some application name/value pairs: user, hostname
        SetupAppEnvironment();

        //SetupRosEnvironment - needs to go before ROS!
        //SetupRosEnvironment(""); // FAILS hard coded env above

        // Initialize ROS
        ros::init(argc, argv, ROSPACKAGENAME);
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
        path = ros::package::getPath(ROSPACKAGENAME);
        Globals._appproperties[ROSPACKAGENAME] = path;
        MotionException::Load();
        Nist::Config cfg;
        cfg.load(path + "/config/config.ini");
        std::string appname = cfg.GetSymbolValue("system.name", "").c_str();

        std::vector<std::string> robots = cfg.GetTokens("system.robots", ",");
        std::vector<double> ds;

        std::vector<boost::shared_ptr<CController> > ncs;
        std::vector<InlineRobotCommands > nccmds;
        for (size_t i = 0; i < robots.size(); i++) {

            std::string robotname = cfg.GetSymbolValue(robots[i] + ".longname", "robot").c_str();
            double dCycleTime = cfg.GetSymbolValue(robots[i] + ".cycletime", "robot").toNumber<double>();
            std::string prefix = cfg.GetSymbolValue(robots[i] + ".prefix", "").c_str();
            std::string eelink = cfg.GetSymbolValue(robots[i] + ".eelink", "").c_str();
            std::string baselink = cfg.GetSymbolValue(robots[i] + ".baselink", "").c_str();
            std::vector<double> dbase = cfg.GetDblTokens(robots[i] + ".base", ",");
            std::vector<double> dtool = cfg.GetDblTokens(robots[i] + ".tool", ",");
            std::vector<double> dbend = cfg.GetDblTokens(robots[i] + ".bend", ",");
            std::string kinsolver = cfg.GetSymbolValue(robots[i] + ".kinsolver", "").c_str();
            std::vector<std::string> jointmovenames = cfg.GetTokens(robots[i] + ".jointmovenames", ",");
            int bCsvLogging = cfg.GetSymbolValue(robots[i] + ".csvlogging", "0").toNumber<int>();
            // Fixme: add some sanity checking

            ncs.push_back(boost::shared_ptr<CController>(new RCS::CController(robotname, dCycleTime)));
            ncs[i]->SetToolOffset(Conversion::CreatePose(dtool));
            ncs[i]->SetBaseOffset(Conversion::CreatePose(dbase));
            ncs[i]->QBend() = tf::Quaternion(Deg2Rad(dbend[0]), Deg2Rad(dbend[1]), Deg2Rad(dbend[2]));
            ncs[i]->bCvsPoseLogging() = false;
            boost::shared_ptr<IKinematics> kin;
            ncs[i]->CycleTime() = dCycleTime;

            if (kinsolver == "FanucLRMate200idFastKinematics")
                kin = boost::shared_ptr<IKinematics>(new FanucLRMate200idFastKinematics(ncs[i]));
            if (kinsolver == "MotomanSia20dFastKinematics")
                kin = boost::shared_ptr<IKinematics>(new MotomanSia20dFastKinematics(ncs[i]));
            kin->Init(std::string("manipulator"), eelink, baselink);
            kin->Init(nh);
            ncs[i]->Kinematics() = kin;
            ncs[i]->Setup(nh, prefix);

            // This should be selectable
            ncs[i]->_interpreter = boost::shared_ptr<IRCSInterpreter>(new RCS::BangBangInterpreter(ncs[i], kin));

            //    ncs[i]->NamedJointMove["Safe"] = ToVector<double>(6, 1.49, -0.17, -1.14, 0.11, -0.45, -1.67);
            for (size_t j = 0; j < jointmovenames.size(); j++) {
                ds = cfg.GetDblTokens(robots[i] + "." + jointmovenames[j], ",");
                ncs[i]->NamedJointMove[jointmovenames[j]] = ds;
            }
            nccmds.push_back(InlineRobotCommands(ncs[i])); // , fanuchints);

            LOG_DEBUG << "NC " << ncs[i]->Name().c_str();
            LOG_DEBUG << "base link " << ncs[i]->Kinematics()->getRootLink().c_str();
            LOG_DEBUG << "ee link " << ncs[i]->Kinematics()->getTipLink().c_str();
            LOG_DEBUG << "num joints " << ncs[i]->Kinematics()->NumJoints();
            LOG_DEBUG << "baseoffset " << RCS::DumpPoseSimple(ncs[i]->basePose()).c_str();
            LOG_DEBUG << "tooloffset " << RCS::DumpPoseSimple(ncs[i]->gripperPose()).c_str();
            LOG_DEBUG << "safe " << VectorDump<double>(ncs[i]->NamedJointMove["Safe"]).c_str();
            LOG_DEBUG << "Joint names " << VectorDump<std::string>(ncs[i]->Kinematics()->JointNames()).c_str();
            //LOG_DEBUG << "cycletime " << ncs[i]->Name();

        }

        InitScene();
        DrawScene(); // Debug: LOG_DEBUG << ObjectDB::DumpDB();

        // Move robots to "safe position"
        for (size_t j = 0; j < ncs.size(); j++) {
            ncs[j]->Start(); // start the Controller Session thread
            nccmds[j].MoveJoints(ncs[j]->Kinematics()->AllJointNumbers(), ncs[j]->NamedJointMove["Safe"]);

        }

        ClearScene();
        NewScene();
        DrawScene();
        
#if 1
        // This waits for a publish point from rviz before proceeding
        while (!rvizdemo.Ready()) {
            ros::Duration(1.0).sleep();
        }
        ros::Duration(5.0).sleep();
#endif
        //spinner.stop();
        //      ros::spin();
        do {
            for (size_t i = 0; i < ncs.size(); i++) {
                if (ncs[i]->crclcmds.SizeMsgQueue() == 0) {
                    nccmds[i].TestRobotCommands();
                }
                while (ncs[i]->IsBusy()) {
                    ros::spinOnce();
                    ros::spinOnce();
                    r.sleep();
                }
                ClearScene();
                NewScene();
                DrawScene();
           }

#ifdef BOLTDEMO
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


