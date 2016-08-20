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
#include "RvizMarker.h"

// No namespace declarations
//////////////////////////////////
//ALogger Logger;

#include "BLogging.h"

// RCS namespace declarations
//////////////////////////////////
namespace RCS {
    boost::mutex cncmutex;


    // ----------------------------------------------------
    // Static definitions
    RCS::CController Controller(DEFAULT_LOOP_CYCLE);
    RCS::CanonWorldModel CController::status;
    RCS::CanonWorldModel CController::laststatus;
    RCS::CMessageQueue<RCS::CanonCmd> CController::robotcmds;
    std::list<RCS::CanonCmd> CController::donecmds;
    bool RCS::CController::bSimulation = true;

    RCS::CMessageQueue<nistcrcl::CrclCommandMsg> CController::crclcmds; /**< queue of commands interpreted from Crcl messages */
    //Trajectory CController::trajectory_model;

    // ----------------------------------------------------
    // CController 
    CController::CController(double cycletime) :  RCS::Thread(cycletime) {
        //IfDebug(LOG_DEBUG << "CController::CController"); // not initialized until after main :()
        eJointMotionPlanner = NOPLANNER;
        eCartesianMotionPlanner = NOPLANNER;
 
    }

    CController::~CController(void) {
    }

    bool CController::Verify() {
        IfDebug(LOG_DEBUG << "CController::Verify");
#ifdef  MOVEITKIN
        assert(Kinematics() != NULL);
        assert(TrajectoryModel() != NULL);
#endif
    }
 
    void CController::CmdCallback(const nistcrcl::CrclCommandMsg::ConstPtr& cmdmsg) {
        // "Deep copy" - not sure necessary, but works.
        nistcrcl::CrclCommandMsg cmd(*cmdmsg);
        ROS_INFO("CController::CmdCallback");
        crclcmds.AddMsgQueue(cmd);

    }
    void CController::Setup(ros::NodeHandle &nh) {
        IfDebug(LOG_DEBUG << "CController::Setup");
        Name() = "Controller";

        CController::status.Init();
        _nh=&nh;
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>("crcl_status", 10);
        crcl_cmd = _nh->subscribe("crcl_command", 10, &CController::CmdCallback, this);
        rviz_jntcmd = _nh->advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 10); 
        CController::status.currentjoints = Controller.Kinematics()->ZeroJointState();
        CController::status.currentjoints.name = Controller.Kinematics()->JointNames();
        Controller.gripper.init(nh);
        // fixme: read gripper status
    }

    RCS::CanonCmd CController::GetLastRobotCommand() {
       IfDebug(LOG_DEBUG << "CController::GetLastRobotCommand");
         try {
            RCS::CanonCmd cc = Controller.robotcmds.BackMsgQueue();
            return cc;
        } catch (...) {
            // exception if nothing in queue
        }
        return RCS::Controller.LastCC();
    }

    JointState CController::GetLastJointState() {
      IfDebug(LOG_DEBUG << "CController::GetLastJointState");
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
       IfDebug(LOG_DEBUG << "CController::GetLastCommandedPose");
       JointState lastjoints = GetLastJointState();
	return Controller.Kinematics()->FK(lastjoints.position); /**<  current robot pose */
        //return EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
        //return RCS::Controller.Kinematics()->FK(lastjoints.position);
    }

    void CController::PublishCrclStatus() {
        nistcrcl::CrclStatusMsg statusmsg;
        statusmsg.header.stamp = ros::Time::now();
        statusmsg.crclcommandstatus = Controller.status.crclcommandstatus; // done
        statusmsg.crclcommandnum = Controller.status.echocmd.crclcommandnum;
        statusmsg.crclstatusnum = Controller.status.echocmd.crclcommandnum;
 	Controller.status.currentpose=Controller.Kinematics()->FK(Controller.status.currentjoints.position); /**<  current robot pose */
        ConvertTfPose2GeometryPose(Controller.status.currentpose, statusmsg.statuspose);
        statusmsg.statusjoints = Controller.status.currentjoints;
        statusmsg.eepercent = Controller.status.eepercent;
        crcl_status.publish(statusmsg);
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
 //     IfDebug(LOG_DEBUG << "CController::Action");
        try {
            boost::mutex::scoped_lock lock(cncmutex);
            if(crclcmds.SizeMsgQueue() > 0){
                // Translate into Controller.cmds 
                // FIXME: this is an upcast
				RCS::CanonCmd cc;
				nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                                LOG_DEBUG << "Msg Joints " << VectorDump<double>(msg.joints.position).c_str();
				cc.Set(msg);
                                LOG_DEBUG << "CC Joints " << VectorDump<double>(cc.joints.position).c_str();
#define FEEDBACKTEST2
#ifdef FEEDBACKTEST
				Controller.status.echocmd=cc; /**<  copy of current command */
				Controller.status.currentjoints=Controller.Kinematics()->UpdateJointState(cc.jointnum, Controller.status.currentjoints, cc.joints); 
				Controller.status.currentpose=Controller.Kinematics()->FK(Controller.status.currentjoints.position); /**<  current robot pose */
                                Controller.status.currentjoints.header.stamp = ros::Time(0);
                                LOG_DEBUG << "Current Joints " << VectorDump<double>(Controller.status.currentjoints.position).c_str();
                                rviz_jntcmd.publish(Controller.status.currentjoints);
#elif defined FEEDBACKTEST2
                                Controller.robotcmds.AddMsgQueue(cc);  // ok if not pose
                                //Controller.robotcmds.BackMsgQueue().status = CanonStatusType::CANON_WAITING;
#else                          
				_interpreter.ParseCommand(cc);
#endif
            }

            // Motion commands to robot - only joint at this point
            if (Controller.robotcmds.SizeMsgQueue() == 0) {
                 _lastcc = _newcc;
                _lastcc.status = CanonStatusType::CANON_DONE;
                Controller.status.crclcommandstatus= CanonStatusType::CANON_DONE;
            } else {
                _lastcc = _newcc;
                _newcc = Controller.robotcmds.PopFrontMsgQueue();
                _newcc.status = CanonStatusType::CANON_WORKING;
                // update status
                RCS::Controller.status.echocmd = _newcc;
                Controller.status.crclcommandstatus= CanonStatusType::CANON_WORKING;

                if (_newcc.crclcommand == CanonCmdType::CANON_DWELL) {
                    // This isn't very exact, as there will some time wasted until we are here
                    // Thread cycle time already adjusted to seconds
                    _newcc.dwell_seconds -= ((double) CycleTime());
                    if (_newcc.dwell_seconds > 0.0) {
                        Controller.robotcmds.InsertFrontMsgQueue(_newcc);
                    }
                } else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER) {
                    sensor_msgs::JointState gripperjoints;
                    if (_newcc.eepercent < 1.0)
                        gripperjoints = gripper.closeSetup();
                    else
                        gripperjoints = gripper.openSetup();
                    Controller.status.eepercent =_newcc.eepercent;
                    rviz_jntcmd.publish(gripperjoints);
                    // No speed control for now.
                    
                } else {
#ifdef FEEDBACKTEST2
                    Controller.status.currentjoints = Controller.Kinematics()->UpdateJointState(_newcc.jointnum, Controller.status.currentjoints, _newcc.joints);
#else
                    // Should have been updated by interpreter - and many more of them
                    Controller.status.currentjoints = _newcc.joints;
#endif
                    Controller.status.currentpose = Controller.Kinematics()->FK(Controller.status.currentjoints.position); /**<  current robot pose */
                    Controller.status.currentjoints.header.stamp = ros::Time(0);
                    LOG_DEBUG << "Current Joints " << VectorDump<double>(Controller.status.currentjoints.position).c_str();
                    rviz_jntcmd.publish(Controller.status.currentjoints);
                  
                    //#define MARKERS
#ifdef MARKERS
                    RCS::Pose goalpose=Controller.Kinematics()->FK(_newcc.joint.position);
                    //RCS::Pose goalpose = EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
                    LOG_DEBUG << "Marker Pose " << DumpPose(goalpose).c_str();
                    RvizMarker()->Send(goalpose);
#endif
                }
            }
            PublishCrclStatus();
            // MotionLogging();

        } catch (std::exception & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Exception in CController::Action() thread\n";
        }
        return 1;
    }

 

    // ----------------------------------------------------
    // RobotStatus 

    RobotStatus::RobotStatus(double cycletime) : RCS::Thread(cycletime) {
        Name() = "RobotStatus";
    }

    int RobotStatus::Action() {
        try {
            static int i = 1;
            boost::mutex::scoped_lock lock(cncmutex);
            sensor_msgs::JointState curjoints;
            if (JointReader() != NULL) {
                // Get latest robot  joint readings
                curjoints = JointReader()->GetCurrentReadings();
                if (curjoints.position.size() == 0)
                    return 1;
                assert(curjoints.position.size() != 0);
                RCS::Controller.status.currentjoints = curjoints;
                // Use forward kinematics to get current pose
                Controller.status.currentpose=Controller.Kinematics()->FK(curjoints.position);
                //RCS::Controller.status.currentpose = Kinematics()->FK(cjoints.position);
                //RCS::Controller.status.currentpose = RCS::Controller.EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
               LOG_DEBUG << RCS::DumpPose(RCS::Controller.status.currentpose);
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

