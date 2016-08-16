// Controller.cpp

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

#define BOOST_ALL_NO_LIB

#ifdef WIN32
#define _WIN32_WINNT_WIN7    0x0601
#define _WIN32_WINNT         _WIN32_WINNT_WIN7
#define _WIN32_WINDOWS       _WIN32_WINNT_WIN7
#endif

#include "Controller.h"
#include <boost/exception/all.hpp>
#include <boost/thread.hpp>
#include <strstream>
#include <iostream>

#include "urdf_model/rosmath.h"
//#include "CrclInterface.h"
#include "RvizMarker.h"

// No namespace declarations
//////////////////////////////////
ALogger Logger;



// RCS namespace declarations
//////////////////////////////////
namespace RCS {
    boost::mutex cncmutex;
    static const char *sStateEnums[] = {
        "CRCL_Done", "CRCL_Error", "CRCL_Working", "CRCL_Ready"
    };
#ifdef WIN32

    static void trans_func(unsigned int u, EXCEPTION_POINTERS *pExp) {
        std::stringstream str;

        str << "CController trans_func - Code = " << pExp->ExceptionRecord->ExceptionCode << std::endl;

        OutputDebugString(str.str().c_str());
        throw std::exception();
    }
#else

#endif
    RCS::CController Controller(DEFAULT_LOOP_CYCLE);
    std::vector<std::string>  RCS::CController::links;
    RCS::CanonWorldModel CController::wm;
    RCS::CanonWorldModel CController::status;
    RCS::CanonWorldModel CController::laststatus;
    // RCS::CMessageQueue<RCS::CanonCmd> CController::cmds;
    RCS::CMessageQueue<RCS::CanonCmd> CController::robotcmds;
    RCS::CController::xml_message_list CController::donecmds;
    bool RCS::CController::bSimulation = true;
    std::vector<std::string> CController::joint_names;
    std::vector<std::string> CController::link_names;

    RCS::CMessageQueue<nistcrcl::CrclCommandMsg> CController::crclcmds; /**< queue of commands interpreted from Crcl messages */
    //Trajectory CController::trajectory_model;

    size_t RCS::CController::_NumJoints;

    unsigned long CController::_debuglevel = 0;
    unsigned long CController::_debugtype = (unsigned long) RPY;
    unsigned long CController::_csvlogFlag = 0;
    ALogger CController::CsvLogging;

    CController::CController(double cycletime) :  RCS::Thread(cycletime) {
        eJointMotionPlanner = NOPLANNER;
        eCartesianMotionPlanner = NOPLANNER;
 
    }

    CController::~CController(void) {
    }

    bool CController::Verify() {
#ifdef  MOVEITKIN
        assert(Kinematics() != NULL);
#endif
        assert(TrajectoryModel() != NULL);
    }
 
    void CController::CmdCallback(const nistcrcl::CrclCommandMsg::ConstPtr& cmdmsg) {
        // "Deep copy" - not sure necessary, but works.
        nistcrcl::CrclCommandMsg cmd(*cmdmsg);
        ROS_INFO("CController::CmdCallback");
        crclcmds.AddMsgQueue(cmd);

    }
    void CController::Setup(ros::NodeHandle &nh) {
        Name() = "Controller";
        // CSV Logging setup
        std::string sStatus = DumpHeader(",") + "\n";
        CsvLogging.Timestamping() = false;
        CsvLogging.LogMessage("Timestamp," + sStatus);
        _nh=&nh;
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>("crcl_status", 10);
        crcl_cmd = _nh->subscribe("/crcl_command", 10, &CController::CmdCallback, this);
        armkin=boost::shared_ptr<::Kinematics>( new ::Kinematics());
        armkin->init(nh);
        moveit_msgs::GetKinematicSolverInfo::Request request;
        moveit_msgs::GetKinematicSolverInfo::Response response;
        armkin->getFKSolverInfo(request, response) ;
        joint_names.clear(); link_names.clear();
        _NumJoints=response.kinematic_solver_info.joint_names.size();
        for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++){
             joint_names.push_back(response.kinematic_solver_info.joint_names[i]);
        }
        for(unsigned int i=0; i< response.kinematic_solver_info.link_names.size(); i++){
             link_names.push_back(response.kinematic_solver_info.link_names[i]);
        }
    }

    RCS::CanonCmd CController::GetLastRobotCommand() {
        try {
            RCS::CanonCmd cc = Controller.robotcmds.BackMsgQueue();
            return cc;
        } catch (...) {
            // exception if nothing in queue
        }
        return RCS::Controller.LastCC();
    }

    JointState CController::GetLastJointState() {
        // This assumes queue of motion commands, with last being last in queue.
        try {
            RCS::CanonCmd cc = Controller.robotcmds.BackMsgQueue();
            // could be dwell so no actual joint positions in cmd at this point??
            if(cc.joints.position.size()==0)
                throw std::runtime_error("Zero joint positions\n");
            return cc.joints;
        } catch(std::exception err){
            std::cout << "CController::GetLastJointState exception" << err.what(); 
        } catch (...) {
            // exception if nothing in queue
        }
        // use actual readings of joints
        return RCS::Controller.status.currentjoints;
    }

    RCS::Pose CController::GetLastCommandedPose() {
        JointState lastjoints = GetLastJointState();
        return EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
        //return RCS::Controller.Kinematics()->FK(lastjoints.position);
    }
    int CController::Action() {
        try {
            boost::mutex::scoped_lock lock(cncmutex);
            if(crclcmds.SizeMsgQueue() > 0){
                // Translate into Controller.cmds 
                // FIXME: this is an upcast
                 RCS::CanonCmd cc;
                 nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                 cc.Set(msg);
                _interpreter.ParseCommand(cc);
            }
#if 0
            /////////////////////////////////////////////////////////////////////////////////////////////
            // interpret translated CRCL command. Commands in canonical form: standard units (mm, radians)
            if (Controller.cmds.SizeMsgQueue() > 0) {
                RCS::CanonCmd cc = Controller.cmds.PopFrontMsgQueue();
                _interpreter.ParseCommand(cc);
            }
#endif

            // Motion commands to robot - only joint at this point
            if (Controller.robotcmds.SizeMsgQueue() == 0) {
                 _lastcc = _newcc;
                _lastcc.status = CanonStatusType::CANON_DONE;
            } else {
                _lastcc = _newcc;
                _newcc = Controller.robotcmds.PopFrontMsgQueue();
                _newcc.status = CanonStatusType::CANON_WORKING;
                RCS::Controller.status.echocmd = _newcc;

                if (_newcc.crclcommand == CanonCmdType::CANON_DWELL) {
                    // This isn't very exact, as there will some time wasted until we are here
                    // Thread cycle time already adjusted to seconds
                    _newcc.dwell_seconds -= ((double) CycleTime());
                    if (_newcc.dwell_seconds > 0.0) {
                        Controller.robotcmds.InsertFrontMsgQueue(_newcc);
                    }
                } else {
                    //Controller.trajectory_writer->JointTrajectoryWrite(newcc.joints);
                    Controller.JointWriter()->JointTrajectoryPositionWrite(_newcc.joints);
#define MARKERS
#ifdef MARKERS
                    RCS::Pose goalpose = EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
                    std::cout << "Marker Pose " << DumpPose(goalpose).c_str();
                    RvizMarker()->Send(goalpose);
#endif
                }
            }

            /////////////////////////////////////////////////////////////////////////////////////////////
            // Save status to csv logging file?
            if (CsvLogging.DebugLevel() > INFORM) {
                std::string sStatus = Dump(",") + "\n";

                if (lastlogstatus != sStatus) {
                    CsvLogging.LogMessage(Logger.Timestamp() + "," + sStatus);
                }
                lastlogstatus = sStatus;
            }
        } catch (std::exception & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Exception in CController::Action() thread\n";
        }
        return 1;
    }

    std::string CController::DumpHeader(std::string separator) {
        std::stringstream str;
        const char * fields[] = {
            "CommandID", "StatusID", "State", "Pose-X", "Pose-Y", "Pose-Z",
            "Xrot-I", "Xrot-J", "Xrot-K", "Zrot-I", "Zrot-J", "Zrot-K"
        };
        const char *rpyfields[] = {"CommandID", "StatusID", "State", "Pose-X", "Pose-Y", "Pose-Z", "Roll", "Pitch", "Yaw"};

        if (_debugtype == CRCL) {
            for (size_t i = 0; i < sizeof ( fields) / sizeof ( fields[0]); i++) {
                str << fields[i] << separator.c_str();
            }
        } else {
            for (size_t i = 0; i < sizeof ( rpyfields) / sizeof ( rpyfields[0]); i++) {
                str << rpyfields[i] << separator.c_str();
            }
        }

        for (size_t i = 0; i < 6; i++) {
            str << ((i > 0) ? separator.c_str() : "") << "Joint" << i;
        }
        return str.str();
    }

    std::string CController::Dump(std::string separator) {
        std::stringstream str;

        str.precision(4);

        // str << Globals.GetTimeStamp(CGlobals::GMT_UV_SEC)   << separator.c_str();
 //       str << crclinterface->crclwm.CommandID() << separator.c_str();
 //       str << crclinterface->crclwm.StatusID() << separator.c_str();
        //str << sStateEnums[crclinterface->crclwm.CommandStatus()] << separator.c_str();
 //       str << crclinterface->crclwm.CommandStatus() << separator.c_str();
        //RCS::Pose pose = Crcl::Convert(crclinterface->crclwm._CurrentPose);
 //       str << pose.getOrigin().x() << separator.c_str();
 //       str << pose.getOrigin().y() << separator.c_str();
 //       str << pose.getOrigin().z() << separator.c_str();

        if (_debugtype == CRCL) {
#if 0
            str << crclinterface->crclwm._CurrentPose.XAxis().I() << separator.c_str();
            str << crclinterface->crclwm._CurrentPose.XAxis().J() << separator.c_str();
            str << crclinterface->crclwm._CurrentPose.XAxis().K() << separator.c_str();
            str << crclinterface->crclwm._CurrentPose.ZAxis().I() << separator.c_str();
            str << crclinterface->crclwm._CurrentPose.ZAxis().J() << separator.c_str();
            str << crclinterface->crclwm._CurrentPose.ZAxis().K();
#endif
        } else if (_debugtype == RPY) {
#if 0
            double roll, pitch, yaw;
            getRPY(pose, roll, pitch, yaw);
            //tf::Matrix3x3 rot = pose.getBasis();
            //rot.getRPY(roll, pitch, yaw);
            //pose.rotation.getRPY(roll, pitch, yaw);
            str << roll << separator.c_str();
            str << pitch << separator.c_str();
            str << yaw;
#endif
        }
//        sensor_msgs::JointState joints = Crcl::Convert(crclinterface->crclwm._CurrentJoints);
//        for (size_t i = 0; i < joints.position.size(); i++) {
//            str << separator.c_str() << joints.position[i];
//        }
        return str.str();
    }


    // ----------------------------------------------------
    /**
     *  moveit_msgs::GetPositionIK::Request request;
     *  moveit_msgs::GetPositionIK::Response response;
        request.fk_link_names = linknames;
        request.robot_state.joint_state = joint_state;// # JointState()
        request.robot_state.joint_state.header.frame_id = "base_link";
        request.header.frame_id = "base_link";
     
     */

    RobotStatus::RobotStatus(double cycletime) : RCS::Thread(cycletime) {
        Name() = "RobotStatus";
    }

    int RobotStatus::Action() {
        try {
            static int i = 1;
            boost::mutex::scoped_lock lock(cncmutex);
            sensor_msgs::JointState cjoints;
            if (JointReader() != NULL) {
                // Get latest robot  joint readings
                cjoints = JointReader()->GetCurrentReadings();
                if (cjoints.position.size() == 0)
                    return 1;
                assert(cjoints.position.size() != 0);
                RCS::Controller.status.currentjoints = cjoints;
                // Use forward kinematics to get current pose
                //RCS::Controller.status.currentpose = Kinematics()->FK(cjoints.position);
                RCS::Controller.status.currentpose = RCS::Controller.EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
#ifdef DEBUGCONTROLLERTOOL0POSE
                std::string str= RCS::DumpPose(RCS::Controller.status.currentpose);
                std::cout << str.c_str();
#endif
#if 0
                if (--i < 0) {
                    std::cout << "Current Joints " << VectorDump<double>(cjoints.position).c_str();
                    std::cout << "Canon pose " << DumpPose(RCS::Controller.status.currentpose);
                    std::cout << "Crcl Pose " << Crcl::DumpPose(Crcl::Convert(RCS::Controller.status.currentpose), ",");
                    i = 20;
                }
#endif
                // Now update the CRCL world model which contains latest readings
 //      //         _crclinterface->crclwm.Update(RCS::Controller.status.currentpose);
 //      //         _crclinterface->crclwm.Update(cjoints);
            }
        } catch (...) {
            std::cout << "Exception in RobotStatus::Action()\n";
        }
        return 1;
    }
}

