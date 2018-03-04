

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

#ifdef Qt
#include <QCoreApplication>
#endif
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


#include <nist_robotsnc/NIST/Boost.h>

#include "nist_robotsnc/MotionControl.h"
#include "nist_robotsnc/Globals.h"
#include "nist_robotsnc/Kinematics.h"
#include "nist_robotsnc/Controller.h"
#include "nist_robotsnc/RosSetup.h"
#include "nist_robotsnc/RvizMarker.h"
#include "nist_robotsnc/NIST/Boost.h"
#include "nist_robotsnc/Debug.h"
#include "nist_robotsnc/Scene.h"
#include "nist_robotsnc/RCSInterpreter.h"
#include "nist_robotsnc/Demo.h"
#include "nist_robotsnc/NIST/Config.h"
#include "nist_robotsnc/MotionException.h"
#include "nist_robotsnc/Shape.h"
///#include "nist_robotsnc/ttt.h"

#include "Test.h"

using namespace Conversion;

int main(int argc, char** argv) {
    int bPublishPoint = 0;
    //signal(SIGSEGV, handler);   // install our handler
    try {
        // Find path of executable
        std::string path(argv[0]);
        Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
        Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;
        Globals._appproperties["Package"] = ROSPACKAGENAME;
        Globals._appproperties["PackageSrcPath"] = ROSPACKAGEDIR;

        // FIXME: this totally assume ROS catkin folder layout


#if 0
        std::string pkgname = path.substr(path.find_last_of('/') + 1);
        Globals._appproperties["Package"] = pkgname;
        std::string wspath = path;
        for (size_t i = 0; i < 4; i++)
            wspath = wspath.substr(0, wspath.find_last_of('/'));
        Globals._appproperties["Workspace"] = wspath + "/";

        std::string pkgpath = wspath + "/src/" + pkgname + "/config/" + pkgname + ".ini";
        std::string envstr = ExecuteShellCommand("export ROS_PACKAGE_PATH; cd " + wspath + "; /bin/bash -c \"source devel/setup.bash & env \"");
 //       std::cout << envstr << "\n";
        // This sets up tother application _appproperties name/value pairs: user, hostname
        SetupAppEnpathvironment();
pathpath
        // This hard coding of env variables is required for debugging with netbeans ide
        // If ROS environment variables are not set it cannot find "stuff"
  //      SetupRosEnvironment(); // needs to go before ROS!

  #endif
        //const std::string ROSPACKAGENAME =  pkgname;
        ros::M_string remappings;
        remappings["__master"]="http://localhost:11311";
        remappings["__name"]=ROSPACKAGENAME;//.c_str();
        // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
        //ros::init(argc, &argv[0], ROSPACKAGENAME);
        ros::init(remappings,ROSPACKAGENAME); // .c_str());

        // Check if master running if not abort with dialog
        if(! ros::master::check())
        {
            std::cerr << "ROS Master Not Running - Abrting\n";
            return -1 ;
        }
        
        // Initialize ROS
       // ros::init(argc, argv, ROSPACKAGENAME);
        ros::NodeHandle nh;
        ros::Rate r(50); // 10 times a second - 10Hz

#if 0
        /**
         * The five different ROS verbosity levels are, in order:
         * DEBUG ROS_DEBUG
         * INFO ROS_INFO
         * WARN
         * ERROR
         */
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
#endif
        boostlogfile = nh.param<std::string>("logfile", "/home/isd/michalos/Documents/example.log");
        boostloglevel = (boost::log::v2_mt_posix::trivial::severity_level) nh.param<int>("loglevel", 0); // 0 = debug

        bPublishPoint = nh.param<int>("PublishPoint", 0);

        Globals.DebugSetup();
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

#if 0
        // DOESN"T WORK NOW
        // ROS config - parameter list - save for comparison later
        std::string params = ReadRosParams(nh);
        Globals.WriteFile(Globals.ExeDirectory + "rosconfig.txt", params);
        path = ros::package::getPath(ROSPACKAGENAME);
        Globals._appproperties[ROSPACKAGENAME] = path;
#endif

#ifdef GEARS
        GearDemo geardemo(nh, path, Convert<tf::Vector3, tf::Pose>(tf::Vector3(0.25, 0.5, 0.0)));

        geardemo.Setup(); // this does draw scene
        pScene->DrawScene();
#endif
        // Confetti
#if 0
        for (size_t i = 0; i < 20; i++) {
            double x = ((double) i)*.01;
            Piece * p = new Piece(pScene);
            p->launch(x, 0.0, 1.0);

        }
#endif

        //MotionException::Load();
        std::vector<boost::shared_ptr<CController> > ncs;
        std::vector<CrclApi > nccmds;
        std::vector<double> ds;
        pt::ptree root;
        std::string baselink;

        try {
            // Load the json file in this ptree
            pt::read_ini( Globals._appproperties["PackageSrcPath"] + "/config/" + ROSPACKAGENAME + ".ini", root);
            std::vector<std::string> robots = GetIniTypes<std::string>(root, "system.robots");
            for (size_t i = 0; i < robots.size(); i++) {
                ofsRobotURDF << "============================================================\n";
                std::string robotname = root.get<std::string>(robots[i] + ".longname");
                double dCycleTime = root.get<double>(robots[i] + ".cycletime", 10.0);
                std::string prefix = root.get<std::string>(robots[i] + ".prefix", "");
                std::string eelink = root.get<std::string>(robots[i] + ".eelink", "");
                baselink = root.get<std::string>(robots[i] + ".baselink", "");
                std::vector<double> dbase = GetIniTypes<double>(root, robots[i] + ".base");
                std::vector<double> dtool = GetIniTypes<double>(root, robots[i] + ".tool");
                std::vector<double> dbend = GetIniTypes<double>(root, robots[i] + ".bend");
                std::string kinsolver = root.get<std::string>(robots[i] + ".kinsolver", "");
                std::vector<std::string> jointmovenames = GetIniTypes<std::string>(root, robots[i] + ".jointmovenames");
                int bMarker = root.get<int>(robots[i] + ".markers", 0);
                int bCsvLogging = root.get<int>(robots[i] + ".csvlogging", 0);

                ncs.push_back(boost::shared_ptr<CController>(new RCS::CController(robotname, dCycleTime)));
                ncs[i]->SetGripperOffset(Convert<std::vector<double>, tf::Pose> (dtool));
                ncs[i]->SetBaseOffset(Convert<std::vector<double>, tf::Pose> (dbase));
                ncs[i]->QBend() = tf::Quaternion(Deg2Rad(dbend[0]), Deg2Rad(dbend[1]), Deg2Rad(dbend[2]));
                ncs[i]->bCvsPoseLogging() = false;
                ncs[i]->rotationmax() = GetIniTypes<double>(root, robots[i] + ".rotationmax");
                ncs[i]->linearmax() = GetIniTypes<double>(root, robots[i] + ".linearmax");
                boost::shared_ptr<IKinematics> kin;
                ncs[i]->CycleTime() = dCycleTime;
                ncs[i]->bMarker() = bMarker;

                if (kinsolver == "FanucLRMate200idFastKinematics")
                    kin = boost::shared_ptr<IKinematics>(new FanucLRMate200idFastKinematics(ncs[i]));
                if (kinsolver == "MotomanSia20dFastKinematics") {
                    //kin = boost::shared_ptr<IKinematics>(new MotomanSia20dGoKin(ncs[i]));
                    //kin = boost::shared_ptr<IKinematics>(new MotomanSia20dTrak_IK(ncs[i]));
                    kin = boost::shared_ptr<IKinematics>(new MotomanSia20dFastKinematics(ncs[i]));
                }
                try {
                    kin->Init(std::string("manipulator"), eelink, baselink);
                    kin->Init(nh);
                    ncs[i]->Kinematics() = kin;
                    ncs[i]->Setup(nh, prefix);
                } catch (std::exception & ex) {
                    std::cout << "Kinematics error: " << ex.what() << "\n";
                }

                // This should be selectable
                ncs[i]->Interpreter() = boost::shared_ptr<IRCSInterpreter>(new RCS::BangBangInterpreter(ncs[i], kin));
                //ncs[i]->Interpreter() = boost::shared_ptr<IRCSInterpreter>(new RCS::GoInterpreter(nh, ncs[i], kin));
                for (size_t j = 0; j < jointmovenames.size(); j++) {
                    ds = GetIniTypes<double>(root, robots[i] + "." + jointmovenames[j]);
                    ncs[i]->NamedJointMove[jointmovenames[j]] = ds;
                }
                ncs[i]->Interpreter()->Init(ncs[i]->NamedJointMove["Home"]);
                nccmds.push_back(CrclApi(ncs[i]));
                ofsRobotURDF << "NC " << ncs[i]->Name().c_str() << "\n";
                ofsRobotURDF << "base link " << ncs[i]->Kinematics()->getRootLink().c_str() << "\n";
                ofsRobotURDF << "ee link " << ncs[i]->Kinematics()->getTipLink().c_str() << "\n";
                ofsRobotURDF << "num joints " << ncs[i]->Kinematics()->NumJoints() << "\n";
                ofsRobotURDF << "baseoffset " << RCS::DumpPoseSimple(ncs[i]->basePose()).c_str() << "\n";
                ofsRobotURDF << "tooloffset " << RCS::DumpPoseSimple(ncs[i]->gripperPose()).c_str() << "\n";
                ofsRobotURDF << "Joint names " << VectorDump<std::string>(ncs[i]->Kinematics()->JointNames()).c_str() << "\n" << std::flush;
                ofsRobotURDF << kin->DumpUrdfJoint().c_str() << "\n";
                ofsRobotURDF << kin->DumpTransformMatrices().c_str() << "\n";

                for (std::map<std::string, std::vector<double>>::iterator it = ncs[i]->NamedJointMove.begin(); it != ncs[i]->NamedJointMove.end(); it++)
                    ofsRobotURDF << (*it).first << "=" << VectorDump<double>(ncs[i]->NamedJointMove[(*it).first]).c_str() << "\n";
                //ofsRobotURDF << "cycletime " << ncs[i]->Name();
                ofsRobotURDF << "Cycletime   " << ncs[i]->CycleTime() << "\n";
                ofsRobotURDF << "Markers     " << ncs[i]->bMarker() << "\n";
                ofsRobotURDF << std::flush;

                // Error checking
                if (ncs[i]->linearmax().size() < 3)
                    throw std::runtime_error(Globals.StrFormat("Config error: Insufficient number linear parameters for %s", ncs[i]->Name().c_str()));
                if (ncs[i]->rotationmax().size() < 3)
                    throw std::runtime_error(Globals.StrFormat("Config error: Insufficient number rotation parameters for %s", ncs[i]->Name().c_str()));

            }
        } catch (std::exception &e) {
            LOG_FATAL << e.what();
            throw;
        }
        // This mustbe called first
        pScene->InitScene(nh, baselink);

        double table_length = root.get<double>("checkers.TABLE_LENGTH", 0.4);
        double table_width = root.get<double>("checkers.TABLE_WIDTH", 0.4);
        double table_height = root.get<double>("checkers.TABLE_HEIGHT", 0.05);
        std::vector<double> xyz = GetIniTypes<double>(root, "checkers.TABLE_CENTER");
        tf::Pose centertablepose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(xyz[0], xyz[1], xyz[2]));

        pScene->CreateTable("table1",
                rgba(1.0, 0.0, 0.0, 1.0).GetColorRGBA(),
                "world",
                table_length,
                table_width,
                table_height,
                centertablepose);
        pScene->DrawScene();

#ifdef GRADIENT_COLOR_TABLE
        cColorPicker colorpicker;
        std::vector<rgba> myCols;
        colorpicker.Pick(myCols, 50);
        for (size_t j = 0; j < myCols.size(); j++) {
            pScene->ChangeColor("table1", myCols[j].GetColorRGBA());
            ros::Duration(0.05).sleep();
        }
#endif
#ifdef EXERCISER
        ExerciseDemo exercise(nh);
        IKinematics::_testspacing = root.get<double>("testharness.spacing", 0.5);
        IKinematics::_testoffset = root.get<double>("testharness.offset", 0.1);
        IKinematics::_testepsilon = root.get<double>("testharness.epsilon", 0.1);
        exercise.GoodColor() = GetIniTypes<double>(root, "testharness.GoodColor");
        exercise.BadColor() = GetIniTypes<double>(root, "testharness.BadColor");
        exercise.TrapColor() = GetIniTypes<double>(root, "testharness.TrapColor");
        exercise.Exercise(&nccmds[0]);
        exercise.Exercise(&nccmds[1]);
#endif

#if 0
        std::vector<double> testjts = ToVector<double>(7, 1.30, -0.84, 0.08, 2.26, 2.96, -0.38, -1.28);
        tf::Pose testpose = ncs[1]->Kinematics()->FK(testjts);
        std::cout << "Joint vals " << VectorDump<double>(testjts).c_str() << "\n" << std::flush;
        std::cout << "kinpose " << RCS::DumpPoseSimple(testpose).c_str() << "\n";

        ncs[1]->Kinematics()->axis.push_back(Eigen::Vector3d(0, 0, -1));
        ncs[1]->Kinematics()->xyzorigin.push_back(Eigen::Vector3d(0, 0, 0));
        ncs[1]->Kinematics()->rpyorigin.push_back(Eigen::Vector3d(0, 0, -M_PI_2));
        testjts.push_back(0.0);
        std::vector<tf::Pose> poses = ncs[1]->Kinematics()->ComputeAllFk(testjts);
        std::cout << "genericpose " << RCS::DumpPoseSimple(poses.back()).c_str() << "\n";
#endif      

        // start the Controller Session thread
        for (size_t j = 0; j < ncs.size(); j++) {
            ncs[j]->Start();
        }

        // Move robots to "home position"
        for (size_t j = 0; j < ncs.size(); j++) {
            nccmds[j].MoveJoints(ncs[j]->Kinematics()->AllJointNumbers(), ncs[j]->NamedJointMove["Home"]);
            while (ncs[j]->IsBusy()) {
                ros::spinOnce();
                ros::spinOnce();
                r.sleep();
            }
        }

        // Move robots to "safe position"
        for (size_t j = 0; j < ncs.size(); j++) {
            nccmds[j].MoveJoints(ncs[j]->Kinematics()->AllJointNumbers(), ncs[j]->NamedJointMove["Safe"]);
            while (ncs[j]->IsBusy()) {
                ros::spinOnce();
                ros::spinOnce();
                r.sleep();
            }
        }

        //#define SCRIPTDEMO  
#ifdef SCRIPTDEMO

        pScene->ClearScene();
        tf::Pose penloc(tf::QIdentity(), tf::Vector3(0.0, 0.0, .12));
        ScriptingDemo scriptdemo(nh);
        //scriptdemo.RvizMarker()->SetColor(0.,0.,1.0,1.0);

        std::string penholderfile = root.get<std::string>("stl.holder", "");
        double penholderscale = root.get<double>("stl.holder/scale", 1.0);

        std::string pencilfile = root.get<std::string>("stl.pencil", "");
        double pencilscale = root.get<double>("stl.pencil/scale", 1.0);
        //        scriptdemo.RvizMarker()->publishMesh(
        //                //tf::Pose(tf::AxisAngle(M_PI, tf::XAxis()), tf::Vector3(0.0, 0., 0.5)), // tf::Identity(),
        //                tf::Pose(tf::QIdentity(), tf::Vector3(-0.0144,0.0134,0.0) ), // tf::Identity(),
        //                pencilfile,
        //                pencilscale);


        scriptdemo.Init(&nccmds[0],
                penholderfile,
                penholderscale,
                pencilfile,
                pencilscale);

        tf::Pose rot(tf::AxisAngle(0.5 * M_PI, tf::ZAxis()), tf::Vector3(0.0, 0., 0.0));


        scriptdemo.Draw("NIST", 12, .25, tf::Vector3(0.25, -0.2, 0.25));

#endif

        //#define PAINTDEMO  
#ifdef PAINTDEMO        
        PaintingDemo paintdemo(nh);
        ;
        std::string jpgfile = Globals._appproperties["Workspace"] + "/src/" + Globals._appproperties["Package"] + "/config/american-flag-medium.jpg";
        paintdemo.Draw(&nccmds[0], jpgfile, .25);
#endif
#ifdef CHECKERS
        CheckersGame checkers(nh);
        /** CHeckers board rviz dimensions */
        checkers.RvizGame()->HEIGHT = root.get<double>("checkers.HEIGHT", 0.015);
        checkers.RvizGame()->XOFFSET = root.get<double>("checkers.XOFFSET", -0.16);
        checkers.RvizGame()->YOFFSET = root.get<double>("checkers.YOFFSET", -0.20);
        checkers.RvizGame()->ZOFFSET = root.get<double>("checkers.ZOFFSET", 0.0);
        checkers.RvizGame()->SQOFFSET = root.get<double>("checkers.SQOFFSET", 0.04);
        checkers.RvizGame()->RADIUS = root.get<double>("checkers.RADIUS", 0.0255);
        checkers.RvizGame()->BOARD_DIRECTION = root.get<int>("checkers.BOARD_DIRECTION", -1);

        checkers. z = checkers.RvizGame()->ZOFFSET;
        checkers. table_length = root.get<double>("checkers.TABLE_LENGTH", 0.4);
        checkers. table_width = root.get<double>("checkers.TABLE_WIDTH", 0.4);
        checkers. table_height = root.get<double>("checkers.TABLE_HEIGHT", 0.05);
        checkers.xyz = GetIniTypes<double>(root, "checkers.TABLE_CENTER");

        assert(checkers.xyz.size() > 2);


        Checkers::BoardType outboard;
        std::string filename(Globals._appproperties["PackageSrcPath"] + "/config/" + "Checkers.txt");
        Checkers::CheckersGame & game = checkers.RvizGame()->Game();

newgame:
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
        goto newgame;
#endif
#ifdef GEARS
        do {
            for (size_t i = 1; i < 2; i++) { // only motoman
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
#ifdef Qt
    QCoreApplication a(argc, argv);

    return a.exec();
#endif
}


