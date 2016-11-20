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
//#pragma message "Compiling " __FILE__ 

#include "Controller.h"
#include <boost/exception/all.hpp>
#include <boost/thread.hpp>
#include <strstream>
#include <iostream>
#include <csignal>

#include "urdf_model/rosmath.h"
#include "RvizMarker.h"
#include "Debug.h"
#include "Scene.h"
#include "Boost.h"
#include "MotionException.h"

using namespace Conversion;
// RCS namespace declarations
//////////////////////////////////
namespace RCS {
    boost::mutex cncmutex;

    // ----------------------------------------------------
    // Extern definitions
   // boost::shared_ptr<CController> Fnc = boost::shared_ptr<CController>(new RCS::CController("Fanuc CNC", DEFAULT_LOOP_CYCLE));
    //boost::shared_ptr<CController> Mnc = boost::shared_ptr<CController>(new RCS::CController("Motoman CNC", DEFAULT_LOOP_CYCLE));

    // Static definitions
    bool CController::bRvizPubSetup = false;
    ros::Publisher CController::rviz_jntcmd; 

    // ----------------------------------------------------
    // CController 

    CController::CController(std::string name, double cycletime) : _Name(name), RCS::Thread(cycletime) {
        //IfDebug(LOG_DEBUG << "CController::CController"); // not initialized until after main :()
        eJointMotionPlanner = NOPLANNER;
        eCartesianMotionPlanner = NOPLANNER;
        bCvsPoseLogging() = false;
        bMarker() = false;
        bSimulation() = true;
        gripperPose() = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
        invGripperPose() = gripperPose().inverse();

    }

    CController::~CController(void) {
    }

    bool CController::Verify() {
        IfDebug(LOG_DEBUG << "CController::Verify");
        NC_ASSERT(Kinematics() != NULL);
    }

    void CController::CmdCallback(const nistcrcl::CrclCommandMsg::ConstPtr& cmdmsg) {
        // "Deep copy" 
        nistcrcl::CrclCommandMsg cmd(*cmdmsg);
        ROS_INFO("CController::CmdCallback");
        crclcmds.AddMsgQueue(cmd);
    }

    void CController::Setup(ros::NodeHandle &nh, std::string prefix) {
        IfDebug(LOG_DEBUG << "CController::Setup");
        Name() = prefix + "controller";

        status.Init(this);
        _nh = &nh;
        crcl_status = _nh->advertise<nistcrcl::CrclStatusMsg>(prefix+"crcl_status", 1);
        crcl_cmd = _nh->subscribe(prefix+"crcl_command", 10, &CController::CmdCallback, this);
        if (!bRvizPubSetup) {
            rviz_jntcmd = _nh->advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 1);
            bRvizPubSetup = true;
        }

        status.currentjoints = RCS::ZeroJointState(Kinematics()->JointNames().size());
        status.currentjoints.name = Kinematics()->JointNames();
        gripper.init(nh, prefix);
        
        // fixme: read arm and gripper joint positions
        if (bCvsPoseLogging()) {
            if (CvsPoseLoggingFile().empty())
                LOG_DEBUG << "Empty CController::Setup CvsPoseLoggingFile";
            PoseLogging().Open(CvsPoseLoggingFile());
        }
    }

    RCS::CanonCmd CController::GetLastRobotCommand() {
        IfDebug(LOG_DEBUG << "CController::GetLastRobotCommand"<< Name().c_str());
        try {
            RCS::CanonCmd cc = robotcmds.BackMsgQueue();
            return cc;
        } catch (...) {
            // exception if nothing in queue
        }
        return LastCC();
    }

    JointState CController::GetLastJointState() {
        IfDebug(LOG_DEBUG << "CController::GetLastJointState"<< Name().c_str());
        // This assumes queue of motion commands, with last being last in queue.
        try {
            RCS::CanonCmd cc = robotcmds.BackMsgQueue();
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
        return status.currentjoints;
    }

    RCS::Pose CController::GetLastCommandedPose() {
        IfDebug(LOG_DEBUG << "CController::GetLastCommandedPose"<< Name().c_str());
        JointState lastjoints = GetLastJointState();
        return Kinematics()->FK(lastjoints.position);
        //return EEPoseReader()->GetLinkValue(links.back());
        //return Kinematics()->FK(lastjoints.position);
    }

    void CController::PublishCrclStatus() {
        nistcrcl::CrclStatusMsg statusmsg;
        statusmsg.header.stamp = ros::Time::now();
        statusmsg.crclcommandstatus = status.crclcommandstatus; // done
        statusmsg.crclcommandnum = status.echocmd.crclcommandnum;
        statusmsg.crclstatusnum = status.echocmd.crclcommandnum;
        status.currentpose = Kinematics()->FK(status.currentjoints.position); /**<  current robot pose */
        statusmsg.statuspose=Convert<tf::Pose, geometry_msgs::Pose>(status.currentpose);
        statusmsg.statusjoints = status.currentjoints;
        statusmsg.eepercent = status.eepercent;
        crcl_status.publish(statusmsg);
    }

    bool CController::IsBusy() {
        return (crclcmds.SizeMsgQueue() > 0 || robotcmds.SizeMsgQueue() > 0);
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
                RCS::CanonCmd cc, newcc;
                
                nistcrcl::CrclCommandMsg msg = crclcmds.PeekFrontMsgQueue();
                //nistcrcl::CrclCommandMsg msg = crclcmds.PopFrontMsgQueue();
                LOG_DEBUG << "Command = " << RCS::sCmd[msg.crclcommand];
                cc.Set(msg);
                LOG_TRACE << "Msg Joints " << RCS::VectorDump<double>(msg.joints.position).c_str();
                LOG_TRACE << "CC Joints " << RCS::VectorDump<double>(cc.joints.position).c_str();

                int status = _interpreter->ParseCommand(cc, newcc);
                robotcmds.AddMsgQueue(newcc);
                // Signals done with canon command
                if(status== CanonStatusType::CANON_DONE) 
                {
                    crclcmds.PopFrontMsgQueue();
                }
                else if(status== CanonStatusType::CANON_ERROR )
                {
                    crclcmds.ClearMsgQueue();

                }
            }

            // Motion commands to robot - only joint at this point
            if (robotcmds.SizeMsgQueue() == 0) {
                _lastcc.status = CanonStatusType::CANON_DONE;
                status.crclcommandstatus = CanonStatusType::CANON_DONE;
            } else {
                _lastcc = _newcc;

                _newcc = robotcmds.PeekFrontMsgQueue();
                // _newcc = Cnc.robotcmds.PopFrontMsgQueue();
                robotcmds.PopFrontMsgQueue();

                _newcc.status = CanonStatusType::CANON_WORKING;
                // update status
                status.echocmd = _newcc;
                status.crclcommandstatus = CanonStatusType::CANON_WORKING;

                if (_newcc.crclcommand == CanonCmdType::CANON_DWELL) {
                    // This isn't very exact, as there will some time wasted until we are here
                    // Thread cycle time already adjusted to seconds
                    _newcc.dwell_seconds -= ((double) CycleTime());
                    if (_newcc.dwell_seconds > 0.0) {
                        robotcmds.InsertFrontMsgQueue(_newcc);
                    }
                } else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER) {
                    sensor_msgs::JointState gripperjoints;
                    if (_newcc.eepercent == 0.0) {
                        gripperjoints = gripper.closeSetup();
                    } else if (_newcc.eepercent == 1.0)
                        gripperjoints = gripper.openSetup();
                    else
                        gripperjoints = gripper.setPosition(_newcc.eepercent);
                    status.eepercent = _newcc.eepercent;
                    rviz_jntcmd.publish(gripperjoints);
                    rviz_jntcmd.publish(gripperjoints);
                    // No speed control for now.

                } else if (_newcc.crclcommand == CanonCmdType::CANON_ERASE_OBJECT) {
                    Eigen::Affine3d& pose = pScene->FindPose(_newcc.partname);
                    pScene->UpdateScene(_newcc.partname, pose, "CLEAR");
                } else if (_newcc.crclcommand == CanonCmdType::CANON_DRAW_OBJECT) {
                    Eigen::Affine3d pose;
                    tf::poseMsgToEigen(_newcc.finalpose, pose);
                    pScene->UpdateScene(_newcc.partname,
                            pose,
                            pScene->MARKERCOLOR(_newcc.partcolor));
                } else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER_POSE) {
                    gripperPose() = Conversion::Convert<geometry_msgs::Pose, tf::Pose>(_newcc.finalpose);
                    invGripperPose() = gripperPose().inverse();
                }
                else if (_newcc.crclcommand == CanonCmdType::CANON_SET_BASE_POSE) {
                    basePose() = Conversion::Convert<geometry_msgs::Pose, tf::Pose>(_newcc.finalpose);
                    invBasePose() = basePose().inverse();
                }
                else if (_newcc.crclcommand == CanonCmdType::CANON_FEEDHOLD) {
                        
                } else if (_newcc.crclcommand == CanonCmdType::CANON_STOP_MOTION) {
                    LOG_DEBUG << "STOP Controller ";
                }

                else {
                    // Should have been updated by interpreter - and many more of them
                   LOG_DEBUG << "Current Joints " << RCS::VectorDump<double>(_newcc.joints.position).c_str();
                    status.currentjoints = _newcc.joints;
                    status.currentpose = Kinematics()->FK(status.currentjoints.position); /**<  current robot pose */
                    status.currentjoints.header.stamp = ros::Time(0);
                    LOG_DEBUG << "Current Pose " << DumpPoseSimple(status.currentpose).c_str();
#if 0
                    // Fixme
                    std::vector<int> outofbounds;
                    std::string msg;
                    if (Kinematics()->CheckJointPositionLimits(status.currentjoints.position, outofbounds, msg)) {
                        throw MotionException(1000, msg.c_str());
                    }
#endif
                    rviz_jntcmd.publish(status.currentjoints);
                    rviz_jntcmd.publish(status.currentjoints);
                    ros::spinOnce();
                    ros::spinOnce();
#if 1
                    if (!_newcc.partname.empty()) {
                        Eigen::Translation3d trans(_newcc.finalpose.position.x, _newcc.finalpose.position.y, _newcc.finalpose.position.z);
                        Eigen::Affine3d pose = Eigen::Affine3d::Identity() * trans;
                        pScene->UpdateScene(_newcc.partname,
                                pose,
                                pScene->MARKERCOLOR(_newcc.partcolor));
                    }
#endif
                    ros::spinOnce();
                    ros::spinOnce();
                    if (bMarker()) {
                        RCS::Pose goalpose = Kinematics()->FK(_newcc.joints.position);
                        //RCS::Pose goalpose = EEPoseReader()->GetLinkValue(links.back());
                        LOG_DEBUG << "Marker Pose " << DumpPose(goalpose).c_str();
                        RvizMarker()->Send(goalpose);
                    }
                }

            }
#ifdef CRCL
            PublishCrclStatus();
#endif
            if (bCvsPoseLogging())
                MotionLogging();
        } catch (MotionException & e) {
            std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
            std::raise(SIGINT);
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
                status.currentjoints = curjoints;
                // Use forward kinematics to get current pose
                Cnc.status.currentpose = Cnc.Kinematics()->FK(curjoints.position);
                //status.currentpose = Kinematics()->FK(cjoints.position);
                //status.currentpose = EEPoseReader()->GetLinkValue(RCS::Controller.links.back());
                LOG_DEBUG << RCS::DumpPose(status.currentpose);
#if 0
                if (--i < 0) {
                    LOG_DEBUG << "Current Joints " << RCS::VectorDump<double>(cjoints.position).c_str();
                    LOG_DEBUG << "Canon pose " << DumpPose(status.currentpose);
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

