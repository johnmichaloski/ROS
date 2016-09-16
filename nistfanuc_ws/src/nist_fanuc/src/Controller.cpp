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

#include "Controller.h"
#include <boost/exception/all.hpp>
#include <boost/thread.hpp>
#include <strstream>
#include <iostream>

#include "urdf_model/rosmath.h"
#include "RvizMarker.h"
#include "Debug.h"
#include "Scene.h"
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


    // ----------------------------------------------------
    // CController 

    CController::CController(double cycletime) : RCS::Thread(cycletime) {
        //IfDebug(LOG_DEBUG << "CController::CController"); // not initialized until after main :()
        eJointMotionPlanner = NOPLANNER;
        eCartesianMotionPlanner = NOPLANNER;
        bCvsPoseLogging() = false;
        bMarker() = false;
        bSimulation() = true;
        gripperPose = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
        invGripperPose = gripperPose.inverse();

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
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>("crcl_status", 1);
        crcl_cmd = _nh->subscribe("crcl_command", 10, &CController::CmdCallback, this);
        rviz_jntcmd = _nh->advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 1);
        Cnc.status.currentjoints = RCS::ZeroJointState(Cnc.Kinematics()->JointNames().size());
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
    std_msgs/Header headerposition
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
            
            // STOP untested
            // Check for STOP MOTION - interrupts any commands and stops motion and clears message queue
            boost::mutex::scoped_lock lock(cncmutex);
            RCS::CMessageQueue<nistcrcl::CrclCommandMsg>::xml_message_queue_iterator it;
            it = std::find_if(crclcmds.begin(), crclcmds.end(),
                    boost::bind(&nistcrcl::CrclCommandMsg::crclcommand, _1) == RCS::CanonCmdType::CANON_STOP_MOTION);
            if (it != crclcmds.end()) {
                crclcmds.ClearMsgQueue();
            }
            
            // Now, we only do one command at a time
            if (crclcmds.SizeMsgQueue() > 0 && robotcmds.SizeMsgQueue() == 0) {
                // Translate into Cnc.cmds 
                // FIXME: this is an upcast
                RCS::CanonCmd cc;
                nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                LOG_DEBUG << "Command = " << RCS::sCmd[msg.crclcommand];
                cc.Set(msg);
                LOG_TRACE << "Msg Joints " << VectorDump<double>(msg.joints.position).c_str();
                LOG_TRACE << "CC Joints " << VectorDump<double>(cc.joints.position).c_str();
                
//#define FEEDBACKTEST2
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
                _interpreter->ParseCommand(cc);
#endif
            }

            // Motion commands to robot - only joint at this point
            if (Cnc.robotcmds.SizeMsgQueue() == 0) {
                _lastcc.status = CanonStatusType::CANON_DONE;
                Cnc.status.crclcommandstatus = CanonStatusType::CANON_DONE;
            } else {
                _lastcc = _newcc;
                
                _newcc = Cnc.robotcmds.PeekFrontMsgQueue();
              // _newcc = Cnc.robotcmds.PopFrontMsgQueue();
                  Cnc.robotcmds.PopFrontMsgQueue();

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
                    if (_newcc.eepercent == 0.0) {
                        gripperjoints = gripper.closeSetup();
                    } else if (_newcc.eepercent == 1.0)
                        gripperjoints = gripper.openSetup();
                    else 
                        gripperjoints = gripper.setPosition(_newcc.eepercent);
                    Cnc.status.eepercent = _newcc.eepercent;
                    rviz_jntcmd.publish(gripperjoints);
                    rviz_jntcmd.publish(gripperjoints);
                    // No speed control for now.

                }
                else if (_newcc.crclcommand == CanonCmdType::CANON_ERASE_OBJECT) {
                    Eigen::Affine3d& pose = ObjectDB::FindPose(_newcc.partname);
                    UpdateScene(_newcc.partname, pose, rviz_visual_tools::CLEAR);
                } else if (_newcc.crclcommand == CanonCmdType::CANON_DRAW_OBJECT) {
                    Eigen::Affine3d pose;
                    tf::poseMsgToEigen(_newcc.finalpose, pose);
                    UpdateScene(_newcc.partname,
                            pose,
                            _newcc.partcolor);
                } else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER_POSE) {
                    gripperPose = Conversion::GeomMsgPose2RcsPose(_newcc.finalpose);
                    invGripperPose = gripperPose.inverse();
                } else {
#ifdef FEEDBACKTEST2
                    Cnc.status.currentjoints = Cnc.Kinematics()->UpdateJointState(_newcc.jointnum, Cnc.status.currentjoints, _newcc.joints);
#else
                    // Should have been updated by interpreter - and many more of them
                    Cnc.status.currentjoints = _newcc.joints;
#endif
                    Cnc.status.currentpose = Cnc.Kinematics()->FK(Cnc.status.currentjoints.position); /**<  current robot pose */
                    Cnc.status.currentjoints.header.stamp = ros::Time(0);
                    LOG_DEBUG << "Current Pose " << DumpPoseSimple(Cnc.status.currentpose).c_str();
                    LOG_DEBUG << "Current Joints " << VectorDump<double>(Cnc.status.currentjoints.position).c_str();
                    Cnc.Kinematics()->VerifyLimits(Cnc.status.currentjoints.position);
#if 0
                    if (!_newcc.partname.empty()) {
                        Eigen::Affine3d& pose = ObjectDB::FindPose(_newcc.partname);
                        UpdateScene(_newcc.partname, pose, rviz_visual_tools::CLEAR);
                    }
#endif
                    rviz_jntcmd.publish(Cnc.status.currentjoints);
                    rviz_jntcmd.publish(Cnc.status.currentjoints);
                    ros::spinOnce();
                    ros::spinOnce();
#if 1
                    if (!_newcc.partname.empty()) {
                        Eigen::Translation3d trans(_newcc.finalpose.position.x, _newcc.finalpose.position.y, _newcc.finalpose.position.z);
                        //Eigen::Affine3d pose;
                        //tf::poseMsgToEigen(_newcc.finalpose, pose);
                        Eigen::Affine3d pose = Eigen::Affine3d::Identity() * trans;
                        UpdateScene(_newcc.partname,
                                pose,
                                _newcc.partcolor);
                    }
#endif
                    ros::spinOnce();
                    ros::spinOnce();
                    if (bMarker()) {
                        RCS::Pose goalpose = Cnc.Kinematics()->FK(_newcc.joints.position);
                        //RCS::Pose goalpose = EEPoseReader()->GetLinkValue(RCS::Cnc.links.back());
                        LOG_DEBUG << "Marker Pose " << DumpPose(goalpose).c_str();
                        RvizMarker()->Send(goalpose);
                    }
                }

            }
#if 0
            PublishCrclStatus();
#endif
            if (bCvsPoseLogging())
                MotionLogging();
 
        } catch (std::exception & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Exception in CController::Action() thread\n";
        }
        return 1;
    }

    void CController::MotionLogging() {
        if (bCvsPoseLogging())
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

#ifndef PI_2
#define PI_2 1.5707963268
#endif
double chessdwell=1.0;
// Simplistic Testing code
static int crclcommandnum = 1;
RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1));
tf::Quaternion QBend(M_PI / 2.0, 0.0, 0.0);
// Checkers hint
//  ToVector<double>(6, 0.27, 0.5, -0.4, 0.0, 0.0, 0.0);
std::vector<double> hint= ToVector<double> (6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);

void SetGripper(double ee) {
    // set gripper to 0..1
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = ee; 
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void CloseGripper() { SetGripper(0.75); }
void OpenGripper() { SetGripper(0.45); }
      
void AddGripperOffset(){
    // http://robotiq.com/products/adaptive-robot-gripper/
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER_POSE;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(
            RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
            tf::Vector3(0.140, 0.0, -0.017) )); // -0.01156)));
     RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void DoDwell(double dwelltime) {

    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void MoveTo(RCS::Pose pose,std::string objname){
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    if (hint.size() > 0)
        cmd.hint = hint;
    else cmd.hint = ToVector<double>(6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);

    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    if (!objname.empty())
    {
        ObjectDB * obj = ObjectDB::Find(objname);
        cmd.partname=objname;
        cmd.partcolor = obj->color;
        // lookup gripper close amount from object
    }
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void EraseObject(std::string objname) {
   RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_ERASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void MoveObject(std::string objname, RCS::Pose pose, int color){
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    cmd.partcolor = color;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}
void Pick(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();
 
    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    MoveTo(RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    CloseGripper();
    DoDwell(chessdwell);
    hint= ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
    MoveTo(retract * RCS::Pose(QBend, offset),objname);
}

void MoveJoints(std::vector<long unsigned int> jointnum,
        std::vector<double> positions) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = RCS::ZeroJointState(6);
    // These tovectors must match double or long depending on template or wont work
    cmd.joints.position = positions; // ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0);
    cmd.jointnum = jointnum; // ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

}
// fixme: object has to be richer description
void Place(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();
    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
    DoDwell(chessdwell);
    // MoveTo(RCS::Pose(QBend, offset + tf::Vector3(0.0, 0.0, 0.02)));
    MoveTo(RCS::Pose(QBend, offset), objname);
   // DrawObject(objname, pose);
    DoDwell(chessdwell);
    OpenGripper();
    DoDwell(chessdwell);
    MoveTo(retract * RCS::Pose(QBend, offset));

}

// Major problem is the KDL expects a reaonable hint at the joint solution for IK
// And with large jumps in bang-bang control, not realistic yet.
/// Either ik or more trajectory control with spacing between joint updates small so KDL happy
void TestRobotCommands() {
    // Finish queuing commands before handling them....
    boost::mutex::scoped_lock lock(cncmutex);
    static double dwelltime = 3.0;
    std::vector<long unsigned int> vjointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2));
 
    RCS::CanonCmd cmd;

    // Warm up
    MoveJoints(vjointnum, ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    DoDwell(dwelltime);
    MoveJoints(vjointnum, ToVector<double>(6, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    DoDwell(dwelltime);

    RCS::Pose pickpose =  RCS::Pose(QBend, tf::Vector3(0.25, -0.45, 0.04 + 0.015002));
    
    tf::Vector3 offset = pickpose.getOrigin();
 
    // Retract
    hint = ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
    MoveTo(retract * RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    MoveTo(RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    CloseGripper();
    DoDwell(chessdwell);
    hint = ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
    MoveTo(retract * RCS::Pose(QBend, offset), "bolt1");

    hint = ToVector<double> (6, -0.27, 0.8, -0.55, 0.0, 0.0, 0.0);
    RCS::Pose placepose = RCS::Pose(QBend, 
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245));
    Place(placepose, "bolt1");
}