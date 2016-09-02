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
            boost::mutex::scoped_lock lock(cncmutex);
            // Now, we only do one command at a time
            if (crclcmds.SizeMsgQueue() > 0 && robotcmds.SizeMsgQueue() == 0) {
                // Translate into Cnc.cmds 
                // FIXME: this is an upcast
                RCS::CanonCmd cc;
                nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                LOG_DEBUG << "Command = " << RCS::sCmd[msg.crclcommand]; 
                cc.Set(msg);
                LOG_TRACE<< "Msg Joints " << VectorDump<double>(msg.joints.position).c_str();
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
                    {
                        //gripperjoints = gripper.closeSetup();
                        gripperjoints = gripper.setPosition(0.75);
                    }
                    else
                        gripperjoints = gripper.openSetup();
                    Cnc.status.eepercent = _newcc.eepercent;
                    rviz_jntcmd.publish(gripperjoints);
                    rviz_jntcmd.publish(gripperjoints);
                    // No speed control for now.

                } 
                else if (_newcc.crclcommand == CanonCmdType::CANON_ERASE_OBJECT)
                {
                    Eigen::Affine3d& pose = ObjectDB::FindPose(_newcc.partname);
                    UpdateScene(_newcc.partname, pose, rviz_visual_tools::CLEAR );
                }
                else if (_newcc.crclcommand == CanonCmdType::CANON_DRAW_OBJECT)
                {
                     Eigen::Affine3d pose; tf::poseMsgToEigen(_newcc.finalpose, pose);
                     UpdateScene(_newcc.partname,
                            pose,
                            rviz_visual_tools::RED );
                }
                else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER_POSE)
                {
                          gripperPose=  Conversion::GeomMsgPose2RcsPose(_newcc.finalpose);
                          invGripperPose=gripperPose.inverse();
                }
                else {
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
                            
                    rviz_jntcmd.publish(Cnc.status.currentjoints);
                    rviz_jntcmd.publish(Cnc.status.currentjoints);
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

#ifndef PI_2
#define PI_2 1.5707963268
#endif

void TestRobotCommands() {
    static int crclcommandnum = 1;
    static double dwelltime=3.0;
    
    RCS:Pose retract =  RCS::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0, 0.2));
    RCS::CanonCmd cmd;

    //return ; // skip all commands 
    cmd.joints = RCS::ZeroJointState(6);
    // These tovectors must match double or long
    cmd.joints.position = ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0);
    cmd.jointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.crclcommandnum = crclcommandnum++;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
    
    cmd.joints.position = ToVector<double>(6, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0);
    cmd.jointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.crclcommandnum = crclcommandnum++;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

#if 0
    cmd.joints.position = ToVector<double>(6, -1.05, 1.03, 0.0, 0.07, -0.51, 0.0);
    cmd.jointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.crclcommandnum = crclcommandnum++;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
#endif 
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);


    
    
   // cmd.finalpose = Conversion::RcsPose2GeomMsgPose(Cnc.Kinematics()->FK(ToVector<double>(6, -1.05, 1.03, 0.0, 0.07, -0.51, 0.0)));
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.hint = ToVector<double>(6, -1.05, 1.03, 0.0, 0.07, -0.51, 0.0);
    cmd.ConfigMin() = ToVector<double>(6, -PI_2, -PI_2, -PI_2, -PI_2, -PI_2, -PI_2);
    cmd.ConfigMax() = ToVector<double>(6, PI_2, PI_2, PI_2, PI_2, PI_2, PI_2);
///    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(
//            tf::createQuaternionFromRPY(Deg2Rad(-126.967),Deg2Rad(60.5293), Deg2Rad(171.827)), 
//            tf::Vector3(0.364,-0.641,0.165)
//            ));
    // This was computed taking the FK of the joints to reach (so tool is identity)
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.34063, 0.62224, -0.34978, 0.61192), 
            tf::Vector3(0.26319, -0.46395, 0.04+ 0.15002)));
 //   cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.34063, 0.62224, -0.34978, 0.61192),
 //           tf::Vector3(0.26319, -0.46395, 0.04+ 0.015002)));
    
   
    cmd.bStraight = true;
  //  RCS::Cnc.crclcmds.AddMsgQueue(cmd);
 #if 1   
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER_POSE;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
             tf::Vector3(0.09884, 0.0, -0.01156)));
     //      tf::Vector3(0.14041, 0.0, -0.0041)));
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

#endif
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.34063, 0.62224, -0.34978, 0.61192),
            tf::Vector3(0.26319, -0.46395, 0.04+ 0.015002)));
    cmd.bStraight = true;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = 0.0; // close
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
    
#if 1
    cmd.crclcommand = CanonCmdType::CANON_ERASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = "bolt"; 
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.hint = ToVector<double>(6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.34063, 0.62224, -0.34978, 0.61192),
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245+0.04)));
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);


    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = "bolt";
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245)));     
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = 1.0;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);


    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = 1.0; // open
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    // Now retract
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = 1.0;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.hint = ToVector<double>(6, 0.27, 0.5, -0.4, 0.0, 0.0, 0.0);
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(RCS::Pose(tf::Quaternion(0.34063, 0.62224, -0.34978, 0.61192),
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245+0.04))*retract);
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

    
#endif
}