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
//ALogger Logger;

#include "BLogging.h"

// RCS namespace declarations
//////////////////////////////////
namespace RCS {
    boost::mutex cncmutex;


    RCS::CController Controller(DEFAULT_LOOP_CYCLE);
    std::vector<std::string>  RCS::CController::links;
    //RCS::CanonWorldModel CController::wm;
    RCS::CanonWorldModel CController::status;
    RCS::CanonWorldModel CController::laststatus;
    // RCS::CMessageQueue<RCS::CanonCmd> CController::cmds;
    RCS::CMessageQueue<RCS::CanonCmd> CController::robotcmds;
    RCS::CController::xml_message_list CController::donecmds;
    bool RCS::CController::bSimulation = true;

    RCS::CMessageQueue<nistcrcl::CrclCommandMsg> CController::crclcmds; /**< queue of commands interpreted from Crcl messages */
    //Trajectory CController::trajectory_model;


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
#if 0
        // CSV Logging setup
        std::string sStatus = DumpHeader(",") + "\n";
        CsvLogging.Timestamping() = false;
        CsvLogging.LogMessage("Timestamp," + sStatus);
#endif
        CController::status.Init();
        _nh=&nh;
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>("crcl_status", 10);
        crcl_cmd = _nh->subscribe("crcl_command", 10, &CController::CmdCallback, this);
        CController::status.currentjoints = Controller.Kinematics()->ZeroJointState();
#if 0
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
#endif
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
            LOG_DEBUG << "CController::GetLastJointState exception" << err.what(); 
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
	/**
	std_msgs/Header header

	uint8 crclcommandnum
	uint8 crclstatusnum
	uint8 crclcommandstatus
	################
	uint8 done=0
	uint8 error=1
	uint8 working=2
	################

	geometry_msgs/Pose  statuspose
	sensor_msgs/JointState statusjoints
	float64 eepercent

	*/
    int CController::Action() {
        try {
            boost::mutex::scoped_lock lock(cncmutex);
            if(crclcmds.SizeMsgQueue() > 0){
                // Translate into Controller.cmds 
                // FIXME: this is an upcast
				RCS::CanonCmd cc;
				nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
				cc.Set(msg);
#define FEEDBACKTEST
#ifdef FEEDBACKTEST
				Controller.status.echocmd=cc; /**<  copy of current command */
				Controller.status.currentjoints=Controller.Kinematics()->UpdateJointState(cc.jointnum, Controller.status.currentjoints, cc.joints); 
				Controller.status.currentpose=Controller.Kinematics()->FK(cc.joints.position); /**<  current robot pose */
                                
				nistcrcl::CrclStatusMsg statusmsg;
				statusmsg.header.stamp = ros::Time::now();
				statusmsg.crclcommandstatus=0; // done
				statusmsg.crclcommandnum=cc.crclcommandnum;
				statusmsg.crclstatusnum = cc.crclcommandnum;
				ConvertTfPose2GeometryPose(Controller.status.currentpose, statusmsg.statuspose );
				statusmsg.statusjoints = Controller.status.currentjoints;
				statusmsg.eepercent = cc.eepercent;
				crcl_status.publish(statusmsg);

#else                
				_interpreter.ParseCommand(cc);
#endif
            }

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
                    LOG_DEBUG << "Marker Pose " << DumpPose(goalpose).c_str();
                    RvizMarker()->Send(goalpose);
#endif
                }
            }

			// MotionLogging();

        } catch (std::exception & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Exception in CController::Action() thread\n";
        }
        return 1;
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
               LOG_DEBUGt << str.c_str();
#endif
#if 0
                if (--i < 0) {
                    LOG_DEBUG << "Current Joints " << VectorDump<double>(cjoints.position).c_str();
                    LOG_DEBUG << "Canon pose " << DumpPose(RCS::Controller.status.currentpose);
                    i = 20;
                }
#endif
             }
        } catch (...) {
            LOG_DEBUG << "Exception in RobotStatus::Action()\n";
        }
        return 1;
    }
}

