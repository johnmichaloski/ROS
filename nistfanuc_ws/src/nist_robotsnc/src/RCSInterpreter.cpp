// RCSInterpreter.cpp

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

#include <algorithm>  // max
#include <tf/transform_datatypes.h>
#include <boost/ref.hpp>

#include "RCSInterpreter.h"
#include "Controller.h"
#include "Globals.h"
//#include "trajectoryMaker.h"
#include "Conversions.h"
#include "Debug.h"
#include "nist_robotsnc/MotionException.h"


#define WORLDLOG  std::cout
//#define WORLDLOG  ofsRobotMoveTo
#define DebugWorldCommand

using namespace RCS;
using namespace sensor_msgs;

BangBangInterpreter::BangBangInterpreter(boost::shared_ptr<RCS::CController> nc,
        IKinematicsSharedPtr k)
: _nc(nc), _kinematics(k) {
}

void BangBangInterpreter::SetRange(std::vector<double> minrange, std::vector<double> maxrange) {
    this->minrange = minrange;
    this->maxrange = maxrange;
}

int BangBangInterpreter::ParseCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
        RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus) {

    try {
        if (cmd.crclcommand == CanonCmdType::CANON_MOVE_JOINT) {
            // Move immediately to joint value - Fill in other joints with current values
            cmd.joints = _kinematics->UpdateJointState(cmd.jointnum, _nc->status.currentjoints, cmd.joints);
        } else if (cmd.crclcommand == CanonCmdType::CANON_MOVE_TO) {
            // FIXME: need to subtract off tool offset from tcp
            RCS::Pose finalpose = Conversion::Convert<geometry_msgs::Pose, tf::Pose>(cmd.finalpose);
            RCS::Pose goalpose = finalpose * _nc->invGripperPose();

            LOG_DEBUG << "Final Pose " << RCS::DumpPoseSimple(finalpose).c_str();
            LOG_DEBUG << "Minus Gripper Pose " << RCS::DumpPoseSimple(goalpose).c_str();

            // KDL can handle base offset in fanuc, doesn't even work in Motoman AND needs hints?
            // ikfast solution based on 0,0,0 origin not base offset origin
            goalpose = _nc->invBasePose() * goalpose;
            LOG_DEBUG << " Base Robot Pose " << RCS::DumpPoseSimple(_nc->basePose()).c_str();
            LOG_DEBUG << "Pre Base Robot Pose " << RCS::DumpPoseSimple(goalpose).c_str();
            //cmd.joints.position = Cnc.Kinematics()->IK(goalpose, cmd.ConfigMin(), cmd.ConfigMax());
            cmd.joints.position = _kinematics->IK(goalpose, Subset(_nc->status.currentjoints.position, _nc->Kinematics()->NumJoints()));


            assert(cmd.joints.position.size() > 0);
            LOG_DEBUG << "IK Joints " << VectorDump<double>(cmd.joints.position).c_str();
            cmd.joints.name = _kinematics->JointNames();
            cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;

        }
        //_nc->robotcmds.AddMsgQueue(cmd);
    } catch (MotionException & e) {
        LOG_DEBUG << "Exception in  BangBangInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    }
    outcmd = cmd;
    return CanonStatusType::CANON_DONE;

}
////////////////////////////////////////////////////////////////////////

GoInterpreter::GoInterpreter(boost::shared_ptr<RCS::CController> nc,
        IKinematicsSharedPtr k)
: _nc(nc), _kinematics(k), _lastcmdid(-1) {
    _go = boost::shared_ptr<GoTraj> (new GoTraj());
}

void GoInterpreter::Init(std::vector<double> initjts) {
    JointState jts = RCS::EmptyJointState(initjts.size());
    jts.position = initjts;
    _go->Init(jts, this->_nc->CycleTime());
}

void GoInterpreter::SetRange(std::vector<double> minrange, std::vector<double> maxrange) {
    this->minrange = minrange;
    this->maxrange = maxrange;
}

int GoInterpreter::ParseJointCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
        RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus) {
    try {
        cmd.joints = _kinematics->UpdateJointState(cmd.jointnum, instatus.currentjoints, cmd.joints);

#if defined(DEBUG)  &&  defined(DebugJointCommand)
        ofsRobotMoveJoint << _nc->Name().c_str() << "CANON_MOVE_JOINT: " << "\n";
        ofsRobotMoveJoint << "  Current Joints " << RCS::VectorDump<double>(instatus.currentjoints.position).c_str() << "\n";
        ofsRobotMoveJoint << "  Goal Joints " << RCS::VectorDump<double>(cmd.joints.position).c_str() << "\n";
        ofsRobotMoveJoint << "  Command Num " << cmd.CommandNum() << "\n" << std::flush;
#endif
        std::vector<gomotion::GoTrajParams> jparams  (_kinematics->NumJoints(), gomotion::GoTrajParams(1.0, 10.0, 100.0));
        if (cmd.CommandNum() != _lastcmdid) {
            _lastcmdid = cmd.CommandNum();
            _go->InitJoints(instatus.currentjoints, 
                    cmd.joints, 
                    jparams); // 1 meter/sec

        }

        outcmd = cmd;
        outcmd.joints = _go->NextJoints();
#if defined(DEBUG)    &&  defined(DebugJointCommand)
        LOG_DEBUG << "  Next Joints " << RCS::VectorDump<double>(outcmd.joints.position).c_str();
#endif 
    } catch (MotionException & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    } catch (std::exception & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    }

    if (_go->IsDone())
        return CanonStatusType::CANON_DONE;
    else
        return CanonStatusType::CANON_WORKING;
}

int GoInterpreter::ParseWorldCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
        RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus) {
    try {
        tf::Pose finalpose = Conversion::Convert<geometry_msgs::Pose, tf::Pose>(cmd.finalpose);
        // Need to subtract off tool offset from robot wrist or final tcp
        //tf::Pose goalpose = finalpose * _nc->invGripperPose();
        // Need to translate goal pose from world coordinates into robot coordinates
        //goalpose = _nc->invBasePose() * goalpose;
        //tf::Pose goalpose = finalpose;
        tf::Pose lastpose=_kinematics->FK(_nc->status.currentjoints.position);
        tf::Pose curpose=_nc->basePose() *
                     lastpose *
                    _nc->gripperPose();
        if (cmd.CommandNum() != _lastcmdid) {
            _lastcmdid = cmd.CommandNum();
            _go->InitPose(curpose, 
                    finalpose, 
                    gomotion::GoTrajParams(1.0, 10.0, 100.0),
                    gomotion::GoTrajParams(.1, 1.0, 10.0)); // 1 meter/sec

        }
        //tf::Pose nextpose = _nc->invGripperPose() * _go->NextPose() * _nc->invBasePose();
        //tf::Pose goalpose =_nc->invGripperPose() * finalpose * _nc->invBasePose();
        //tf::Pose nextpose = _nc->invBasePose() * _go->NextPose() * _nc->invGripperPose();
        tf::Pose goalpose =_nc->invBasePose() * finalpose * _nc->invGripperPose();
        tf::Pose gopose =  _go->NextPose() ;
        tf::Pose nextpose = _nc->invBasePose() * gopose * _nc->invGripperPose();
        
        WORLDLOG << _nc->Name().c_str() << ": CANON_MOVE_TO" << "\n";
        WORLDLOG << "WORLD COORDINATES\n";
        WORLDLOG << "    Final Pose    = " << RCS::DumpPoseSimple(finalpose).c_str() << "\n";
        WORLDLOG << "    Cur  Pose     = " << RCS::DumpPoseSimple(curpose).c_str() << "\n";
        WORLDLOG << "    Go      Pose  = " << RCS::DumpPoseSimple(gopose).c_str() << "\n";
        WORLDLOG << "ROBOT COORDINATES\n";
        WORLDLOG << "    GoalRobot Pose= " << RCS::DumpPoseSimple(goalpose).c_str() << "\n";
        WORLDLOG << "    Current   Pose= " << RCS::DumpPoseSimple(lastpose).c_str() << "\n";
        WORLDLOG << "    NextRobot Pose= " << RCS::DumpPoseSimple(nextpose).c_str() << "\n";
 
        // ikfast solution based on 0,0,0 origin not base offset origin
        //cmd.joints.position = Cnc.Kinematics()->IK(goalpose, cmd.ConfigMin(), cmd.ConfigMax());
        JointState goaljoints;
        goaljoints.position = _kinematics->IK(goalpose, Subset(_nc->status.currentjoints.position, _nc->Kinematics()->NumJoints()));
        cmd.joints.position = _kinematics->IK(nextpose, Subset(_nc->status.currentjoints.position, _nc->Kinematics()->NumJoints()));
         _kinematics->IK(lastpose, Subset(_nc->status.currentjoints.position, _nc->Kinematics()->NumJoints()));
        WORLDLOG << "    Goal Joints     =" << VectorDump<double>(goaljoints.position).c_str() << "\n";
        WORLDLOG << "    Current Joints  =" << VectorDump<double>(_nc->status.currentjoints.position).c_str() << "\n";
        WORLDLOG << "    Commanded Joints=" << VectorDump<double>(cmd.joints.position).c_str() << "\n";

#ifdef IKTEST
        std::vector<std::vector<double> > newjoints;
        size_t k = _kinematics->AllIK(nextpose, newjoints);
        WORLDLOG << _nc->Name().c_str() << ": CANON_MOVE_TO" << "\n";
        for (size_t j = 0; j < k; j++)
            WORLDLOG << "    IK   Joints   =" << VectorDump<double>(newjoints[j]).c_str() << "\n";
        WORLDLOG << "    Next Joints   =" << VectorDump<double>(cmd.joinParsets.position).c_str() << "\n";
        WORLDLOG << "    Next De Joints=" << FcnVectorDump(cmd.joints.position, RCS::ToDegree).c_str() << "\n";
#endif       
#if defined(DEBUG)    &&  defined(DebugWorldCommand1)
        WORLDLOG << _nc->Name().c_str() << ": CANON_MOVE_TO" << "\n";
        WORLDLOG << "    Final Pose    = " << RCS::DumpPoseSimple(finalpose).c_str() << "\n";
        WORLDLOG << "    Next  Pose    = " << RCS::DumpPoseSimple(nextpose).c_str() << "\n";
        WORLDLOG << "    Goal Joints   =" << VectorDump<double>(goaljoints.position).c_str() << "\n";
        WORLDLOG << "    Current Joints=" << VectorDump<double>(_nc->status.currentjoints.position).c_str() << "\n";
        WORLDLOG << "    Next Joints   =" << VectorDump<double>(cmd.joints.position).c_str() << "\n";
#endif        
        cmd.joints.name = _kinematics->JointNames();
        cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
        outcmd = cmd;
        if (_go->IsDone())
            return CanonStatusType::CANON_DONE;
        else
            return CanonStatusType::CANON_WORKING;


    } catch (MotionException & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    } catch (std::exception & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    }
}

int GoInterpreter::ParseStopCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
        RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus) {
    try {

        // FIXME: save last command type, then do until done
        if (cmd.CommandNum() != _lastcmdid) {
            _lastcmdid = cmd.CommandNum();
            _go->InitStop();
        }
        return CanonStatusType::CANON_STOP;
        // ??????????????????
    } catch (MotionException & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    } catch (std::exception & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    }
}

int GoInterpreter::ParseCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd,
        RCS::CanonWorldModel instatus, RCS::CanonWorldModel &outstatus) {

    try {
        if (cmd.crclcommand == CanonCmdType::CANON_MOVE_JOINT) {
            return ParseJointCommand(cmd, outcmd, instatus, outstatus);
        } else if (cmd.crclcommand == CanonCmdType::CANON_MOVE_TO) {
            return ParseWorldCommand(cmd, outcmd, instatus, outstatus);
        } else if (cmd.crclcommand == CanonCmdType::CANON_STOP_MOTION) {
            return ParseStopCommand(cmd, outcmd, instatus, outstatus);
        } else if (cmd.crclcommand == CanonCmdType::CANON_DWELL ||
                cmd.crclcommand == CanonCmdType::CANON_SET_GRIPPER ||
                cmd.crclcommand == CanonCmdType::CANON_OPEN_GRIPPER ||
                cmd.crclcommand == CanonCmdType::CANON_CLOSE_GRIPPER) {
            outcmd = cmd;
            return CanonStatusType::CANON_DONE;
        } else if (cmd.crclcommand == CanonCmdType::CANON_DRAW_OBJECT ||
                cmd.crclcommand == CanonCmdType::CANON_ERASE_OBJECT ||
                cmd.crclcommand == CanonCmdType::CANON_GRASP_OBJECT ||
                cmd.crclcommand == CanonCmdType::CANON_RELEASE_OBJECT) {
            outcmd = cmd;
            return CanonStatusType::CANON_DONE;
        }
    } catch (MotionException & e) {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        cmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        cmd.opmessage = e.what();
        cmd.stoptype = CanonStopMotionType::NORMAL;
        outcmd = cmd;
        return CanonStatusType::CANON_ERROR;
    }
    // Eventually should never get here 
    return CanonStatusType::CANON_NOTIMPLEMENTED;
}