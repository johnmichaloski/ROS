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
    // Extern definitions
    RCS::CController Cnc(DEFAULT_LOOP_CYCLE);

    // Static definitions
#if 0
    RCS::CanonWorldModel CController::status;
    RCS::CanonWorldModel CController::laststatus;
    RCS::CMessageQueue<RCS::CanonCmd> CController::robotcmds; /**< queue of commands interpreted into robot motion */
    RCS::CMessageQueue<nistcrcl::CrclCommandMsg> CController::crclcmds; /**< queue of commands from Crcl messages */
    std::list<RCS::CanonCmd> CController::donecmds;
#endif
    //Trajectory CController::trajectory_model;

    // ----------------------------------------------------
    // CController 

    CController::CController(double cycletime) : RCS::Thread(cycletime) {
        //IfDebug(LOG_DEBUG << "CController::CController"); // not initialized until after main :()
        eJointMotionPlanner = NOPLANNER;
        eCartesianMotionPlanner = NOPLANNER;
        bCvsPoseLogging() = false;
        bMarker() = false;
        bSimulation() = true;
    }

    CController::~CController(void) {
    }

    bool CController::Verify() {
        IfDebug(LOG_DEBUG << "CController::Verify");
        assert(Kinematics() != NULL);
#ifdef  MOVEITKIN
        assert(TrajectoryModel() != NULL);
#endif
    }

    void CController::CmdCallback(const nistcrcl::CrclCommandMsg::ConstPtr& cmdmsg) {
        // "Deep copy" 
        nistcrcl::CrclCommandMsg cmd(*cmdmsg);
        ROS_INFO("CController::CmdCallback");
        crclcmds.AddMsgQueue(cmd);

    }

    void CController::Setup(ros::NodeHandle &nh) {
        IfDebug(LOG_DEBUG << "CController::Setup");
        Name() = "Controller";

        status.Init();
        _nh = &nh;
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>("crcl_status", 10);
        crcl_cmd = _nh->subscribe("crcl_command", 10, &CController::CmdCallback, this);
        rviz_jntcmd = _nh->advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 10);
        Cnc.status.currentjoints = Cnc.Kinematics()->ZeroJointState();
        Cnc.status.currentjoints.name = Cnc.Kinematics()->JointNames();
        Cnc.gripper.init(nh);
        // fixme: read arm and gripper joint positions
        if (bCvsPoseLogging()) {
            if (CvsPoseLoggingFile().empty())
                LOG_DEBUG << "Empty CController::Setup CvsPoseLoggingFile";
            PoseLogging().Open(CvsPoseLoggingFile());
        }
    }

    RCS::CanonCmd CController::GetLastRobotCommand() {
        IfDebug(LOG_DEBUG << "CController::GetLastRobotCommand");
        try {
            RCS::CanonCmd cc = Cnc.robotcmds.BackMsgQueue();
            return cc;
        } catch (...) {
            // exception if nothing in queue
        }
        return RCS::Cnc.LastCC();
    }

    JointState CController::GetLastJointState() {
        IfDebug(LOG_DEBUG << "CController::GetLastJointState");
        // This assumes queue of motion commands, with last being last in queue.
        try {
            RCS::CanonCmd cc = Cnc.robotcmds.BackMsgQueue();
            // could be dwell so no actual joint positions in cmd at this point??
            if (cc.joints.position.size() == 0)
                throw std::runtime_error("Zero joint positions\n");
            return cc.joints;
        } catch (std::exception err) {
            LOG_DEBUG << "CController::GetLastJointState exception" << err.what();
        } catch (...) {
            // exception if nothing in queue
        }
        // use actual readings of joints
        return RCS::Cnc.status.currentjoints;
    }

    RCS::Pose CController::GetLastCommandedPose() {
        IfDebug(LOG_DEBUG << "CController::GetLastCommandedPose");
        JointState lastjoints = GetLastJointState();
        return Cnc.Kinematics()->FK(lastjoints.position);
        //return EEPoseReader()->GetLinkValue(RCS::Cnc.links.back());
        //return RCS::Cnc.Kinematics()->FK(lastjoints.position);
    }

    void CController::PublishCrclStatus() {
        nistcrcl::CrclStatusMsg statusmsg;
        statusmsg.header.stamp = ros::Time::now();
        statusmsg.crclcommandstatus = Cnc.status.crclcommandstatus; // done
        statusmsg.crclcommandnum = Cnc.status.echocmd.crclcommandnum;
        statusmsg.crclstatusnum = Cnc.status.echocmd.crclcommandnum;
        Cnc.status.currentpose = Cnc.Kinematics()->FK(Cnc.status.currentjoints.position); /**<  current robot pose */
        Conversion::TfPose2GeometryPose(Cnc.status.currentpose, statusmsg.statuspose);
        statusmsg.statusjoints = Cnc.status.currentjoints;
        statusmsg.eepercent = Cnc.status.eepercent;
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
            // Now, we only do one command at a time
            if (crclcmds.SizeMsgQueue() > 0 && robotcmds.SizeMsgQueue() == 0) {
                // Translate into Cnc.cmds 
                // FIXME: this is an upcast
                RCS::CanonCmd cc;
                nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                LOG_DEBUG << "Msg Joints " << VectorDump<double>(msg.joints.position).c_str();
                cc.Set(msg);
                LOG_DEBUG << "CC Joints " << VectorDump<double>(cc.joints.position).c_str();
#define FEEDBACKTEST2
#ifdef FEEDBACKTEST
                Cnc.status.echocmd = cc; /**<  copy of current command */
                Cnc.status.currentjoints = Cnc.Kinematics()->UpdateJointState(cc.jointnum, Controller.status.currentjoints, cc.joints);
                Cnc.status.currentpose = Cnc.Kinematics()->FK(Cnc.status.currentjoints.position); /**<  current robot pose */
                Cnc.status.currentjoints.header.stamp = ros::Time(0);
                LOG_DEBUG << "Current Joints " << VectorDump<double>(Cnc.status.currentjoints.position).c_str();
                rviz_jntcmd.publish(Cnc.status.currentjoints);
#elif defined FEEDBACKTEST2
                Cnc.robotcmds.AddMsgQueue(cc); // ok if not pose
#else                          
                _interpreter.ParseCommand(cc);
#endif
            }

            // Motion commands to robot - only joint at this point
            if (Cnc.robotcmds.SizeMsgQueue() == 0) {
                _lastcc = _newcc;
                _lastcc.status = CanonStatusType::CANON_DONE;
                Cnc.status.crclcommandstatus = CanonStatusType::CANON_DONE;
            } else {
                _lastcc = _newcc;
                _newcc = Cnc.robotcmds.PopFrontMsgQueue();
                _newcc.status = CanonStatusType::CANON_WORKING;
                // update status
                Cnc.status.echocmd = _newcc;
                Cnc.status.crclcommandstatus = CanonStatusType::CANON_WORKING;

                if (_newcc.crclcommand == CanonCmdType::CANON_DWELL) {
                    // This isn't very exact, as there will some time wasted until we are here
                    // Thread cycle time already adjusted to seconds
                    _newcc.dwell_seconds -= ((double) CycleTime());
                    if (_newcc.dwell_seconds > 0.0) {
                        Cnc.robotcmds.InsertFrontMsgQueue(_newcc);
                    }
                } else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER) {
                    sensor_msgs::JointState gripperjoints;
                    if (_newcc.eepercent < 1.0)
                        gripperjoints = gripper.closeSetup();
                    else
                        gripperjoints = gripper.openSetup();
                    Cnc.status.eepercent = _newcc.eepercent;
                    rviz_jntcmd.publish(gripperjoints);
                    // No speed control for now.

                } else {
#ifdef FEEDBACKTEST2
                    Cnc.status.currentjoints = Cnc.Kinematics()->UpdateJointState(_newcc.jointnum, Cnc.status.currentjoints, _newcc.joints);
#else
                    // Should have been updated by interpreter - and many more of them
                    Cnc.status.currentjoints = _newcc.joints;
#endif
                    Cnc.status.currentpose = Cnc.Kinematics()->FK(Cnc.status.currentjoints.position); /**<  current robot pose */
                    Cnc.status.currentjoints.header.stamp = ros::Time(0);
                    LOG_DEBUG << "Current Joints " << VectorDump<double>(Cnc.status.currentjoints.position).c_str();
                    rviz_jntcmd.publish(Cnc.status.currentjoints);

                    if (bMarker()) {
                        RCS::Pose goalpose = Cnc.Kinematics()->FK(_newcc.joints.position);
                        //RCS::Pose goalpose = EEPoseReader()->GetLinkValue(RCS::Cnc.links.back());
                        LOG_DEBUG << "Marker Pose " << DumpPose(goalpose).c_str();
                        RvizMarker()->Send(goalpose);
                    }
                }
            }
            PublishCrclStatus();
            if (bCvsPoseLogging())
                MotionLogging();

        } catch (std::exception & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Exception in CController::Action() thread\n";
        }
        return 1;
    }

    void CController::MotionLogging(){
        if(bCvsPoseLogging())
            PoseLogging().MotionLogging(status);
    }

#ifdef ROBOTSTATUS
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
                RCS::Cnc.status.currentjoints = curjoints;
                // Use forward kinematics to get current pose
                Cnc.status.currentpose = Cnc.Kinematics()->FK(curjoints.position);
                //RCS::Cnc.status.currentpose = Kinematics()->FK(cjoints.position);
                //RCS::Cnc.status.currentpose = RCS::Cnc.EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
                LOG_DEBUG << RCS::DumpPose(RCS::Cnc.status.currentpose);
#if 0
                if (--i < 0) {
                    LOG_DEBUG << "Current Joints " << VectorDump<double>(cjoints.position).c_str();
                    LOG_DEBUG << "Canon pose " << DumpPose(RCS::Cnc.status.currentpose);
                    i = 20;
                }
#endif
            }
        } catch (...) {
            LOG_DEBUG << "Exception in RobotStatus::Action()\n";
        }
        return 1;
    }
#endif
}
