

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

#include "nist_robotsnc.h"

#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <boost/property_tree/ini_parser.hpp>
namespace pt = boost::property_tree;
#include <boost/algorithm/string.hpp>

#include <ros/package.h>
#include <ros/console.h>


#include "MotionControl.h"
#include "Globals.h"
#include "Kinematics.h"
#include "Controller.h"
#include "RosSetup.h"
#include "RvizMarker.h"
#include "NIST/Boost.h"
#include "Debug.h"
#include "Scene.h"
#include "Checkerboard.h"
#include "StackTrace.h"
#include "RCSInterpreter.h"
#include "Demo.h"
#include "Test.h"
#include "Config.h"
#include "nist_robotsnc/MotionException.h"
#include "nist_robotsnc/Shape.h"

using namespace Conversion;

int main(int argc, char** argv) {
    int bPublishPoint = 0;
    //signal(SIGSEGV, handler);   // install our handler
    try {
        // Find path of executable
        std::string path(argv[0]);
        Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
        Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;

        // This sets up tother application _appproperties name/value pairs: user, hostname
        SetupAppEnvironment();

        // This hard coding of env variables is required for debugging with netbeans ide
        // If ROS environment variables are not set it cannot find "stuff"
        SetupRosEnvironment();

        Nist::Config cfg;
        cfg.load(Globals.ExeDirectory + ROSPACKAGENAME + ".ini");
        std::string appname = cfg.GetSymbolValue("system.name", "").c_str();
        std::map<std::string, std::string> envmap = cfg.getmap("env");
        SetEnvironmentFromMap(envmap);
        SetupRosEnvironment(); // needs to go before ROS!

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

        pScene->InitScene();
        pScene->BuildScene(); // demo actually builds scene
       
        GearDemo geardemo(nh, path, Convert<tf::Vector3, tf::Pose>(tf::Vector3(0.25,0.5,0.0)));
        geardemo.Setup(); // this does draw scene
        
        pScene->DrawScene();
    
        Globals._appproperties[ROSPACKAGENAME] = path;
        MotionException::Load();
        std::vector<boost::shared_ptr<CController> > ncs;
        std::vector<InlineRobotCommands > nccmds;
        std::vector<double> ds;

        try {
            pt::ptree root;
            // Load the json file in this ptree
            //pt::read_ini(::ExeDirectory()+ ROSPACKAGENAME + ".ini", root);
            pt::read_ini(path + "/config/" + ROSPACKAGENAME + ".ini", root);

            std::vector<std::string> robots = GetIniTypes<std::string>(root, "system.robots");
            for (size_t i = 0; i < robots.size(); i++) {
                std::string robotname = root.get<std::string>(robots[i] + ".longname");
                double dCycleTime = root.get<double>(robots[i] + ".cycletime", 10.0);
                std::string prefix = root.get<std::string>(robots[i] + ".prefix", "");
                std::string eelink = root.get<std::string>(robots[i] + ".eelink", "");
                std::string baselink = root.get<std::string>(robots[i] + ".baselink", "");
                std::vector<double> dbase = GetIniTypes<double>(root, robots[i] + ".base");
                std::vector<double> dtool = GetIniTypes<double>(root, robots[i] + ".tool");
                std::vector<double> dbend = GetIniTypes<double>(root, robots[i] + ".bend");
                std::string kinsolver = root.get<std::string>(robots[i] + ".kinsolver", "");
                std::vector<std::string> jointmovenames = GetIniTypes<std::string>(root, robots[i] + ".jointmovenames");
                int bCsvLogging = root.get<int>(robots[i] + ".csvlogging", 0);

                ncs.push_back(boost::shared_ptr<CController>(new RCS::CController(robotname, dCycleTime)));
                ncs[i]->SetToolOffset(Convert<std::vector<double>, tf::Pose> (dtool));
                ncs[i]->SetBaseOffset(Convert<std::vector<double>, tf::Pose> (dbase));
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
                    ds = GetIniTypes<double>(root, robots[i] + "." + jointmovenames[j]);
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
        } catch (std::exception &e) {
            LOG_FATAL << e.what();
        }

        
        // Move robots to "safe position"
        for (size_t j = 0; j < ncs.size(); j++) {
            ncs[j]->Start(); // start the Controller Session thread
            nccmds[j].MoveJoints(ncs[j]->Kinematics()->AllJointNumbers(), ncs[j]->NamedJointMove["Safe"]);
            while (ncs[j]->IsBusy()) {
                ros::spinOnce();
                ros::spinOnce();
                r.sleep();
            }
        }
#if 1
        CheckersGame checkers(nh);
        Checkers::BoardType outboard;
        std::string filename(path + "/config/" + "Checkers.txt");
        Checkers::CheckersGame & game = checkers.RvizGame()->Game();
#if 0
        // This might be cleaner, but the \r problem from windows is disconcerting
        std::ifstream checkersIss(filename);
         LOG_DEBUG << iss.str().c_str();
        if (!checkersIss.fail()) {
            game.Deserialize(iss, outboard);
            LOG_DEBUG << checkers.RvizGame()->Game().printDisplayFancy(outboard).c_str();
            game.Board() = outboard;
        }
#else
        std::string contents;
        Globals.ReadFile(filename, contents);
        boost::replace_all(contents, "\r", "");

        std::stringstream iss;
        iss << contents;
        game.Deserialize(iss, outboard);
        game.Board() = outboard;
#endif
        checkers.Setup();
        checkers.Play(&nccmds[0], &nccmds[1]);
#endif
#if 0
        do {
            for (size_t i = 1; i < 2; i++) {  // only motoman
           // for (size_t i = 0; i < ncs.size(); i++) {
               geardemo.Cycle(ncs[i], nccmds[i]);
               geardemo.Reset();
            }
            r.sleep();
        } while (ros::ok());
#endif
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


