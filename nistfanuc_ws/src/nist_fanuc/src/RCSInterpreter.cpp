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

#include "RCSInterpreter.h"
#include "Controller.h"
#include "Globals.h"
#include <algorithm>  // max
#include "Conversions.h"
#include "trajectoryMaker.h"
#include <tf/transform_datatypes.h>
#include "Conversions.h"
#include "Debug.h"
using namespace RCS;
using namespace sensor_msgs;
#ifdef DESCARTES
using namespace descartes_core;
using namespace descartes_trajectory;
#endif

/**
    uint8 crclcommand

    # https://github.com/ros/common_msgs 
    geometry_msgs/Pose  finalpose
    geometry_msgs/Pose[] waypoints
    # Below joint info could be  trajectory_msgs/JointTrajectoryPoint
    sensor_msgs/JointState joints
    bool bStraight
    float64   dwell_seconds
    string opmessage
    bool bCoordinated
    float64 eepercent
    CrclMaxProfileMsg[] profile # maximum profile 
 */
void BangBangInterpreter::SetRange(std::vector<double> minrange, std::vector<double> maxrange)
{
    this->minrange=minrange;
    this->maxrange=maxrange;
}
int BangBangInterpreter::ParseCommand(RCS::CanonCmd cmd) {

	if (cmd.crclcommand == CanonCmdType::CANON_MOVE_JOINT) {
		// Move immediately to joint value - Fill in other joints with current values
		cmd.joints = Cnc.Kinematics()->UpdateJointState(cmd.jointnum, Cnc.status.currentjoints, cmd.joints);
	}
	else if(cmd.crclcommand == CanonCmdType::CANON_MOVE_TO)
	{
		// FIXME: need to subtract off tool offset from tcp
                RCS::Pose goalpose = Conversion::GeomMsgPose2RcsPose(cmd.finalpose);
		//cmd.joints.position = Cnc.Kinematics()->IK(goalpose, cmd.ConfigMin(), cmd.ConfigMax());
                 cmd.joints.position = Cnc.Kinematics()->IK(goalpose, Subset(Cnc.status.currentjoints.position,6 ));
                cmd.joints.name=Cnc.Kinematics()->JointNames();
		cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
	}

    RCS::Cnc.robotcmds.AddMsgQueue(cmd);

    return 0;
}
SimpleMotionInterpreter::SimpleMotionInterpreter(IKinematicsSharedPtr pKinematics) {
    _kinematics = pKinematics; // hopefully can be null
    //    _kinematics = IKinematicsSharedPtr(new DummyKinematics());
    rates = IRate(DEFAULT_CART_MAX_VEL, DEFAULT_CART_MAX_ACCEL, DEFAULT_LOOP_CYCLE);
}

SimpleMotionInterpreter::~SimpleMotionInterpreter(void) {
}

void SimpleMotionInterpreter::AddJointCommands(std::vector<JointState > gotojoints) {
    for (size_t i = 0; i < gotojoints.size(); i++) {
        RCS::CanonCmd newcc;
        newcc.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
        newcc.status = CanonStatusType::CANON_WAITING;
        newcc.joints.position = gotojoints[i].position;
#ifdef DEBUG
        std::cout << "New Joint Position " << VectorDump<double>(newcc.joints.position).c_str();
#endif
        newcc.joints.velocity = gotojoints[i].velocity;
        newcc.joints.effort = gotojoints[i].effort;
        Cnc.robotcmds.AddMsgQueue(newcc);
    }
}

std::vector<JointState> SimpleMotionInterpreter::PlanCartesianMotion(std::vector<RCS::Pose> poses) {
    std::vector<JointState> gotojoints;
    if (poses.size() == 0)
        return gotojoints;
#ifdef DEBUG
    std::cout << "Current Pose " << DumpPose(poses[0]).c_str();
    std::cout << "Goal Pose " << DumpPose(poses[poses.size() - 1]).c_str();

    for (size_t i = 0; i < poses.size(); i++) {
        std::cout << "CartesianMotion  Poses " << DumpPose(poses[i]).c_str();
    }
#endif
    if (RCS::Cnc.eCartesianMotionPlanner == RCS::CController::MOVEIT) {
#ifdef MOVEITPLANNER
        if (RCS::Cnc.MoveitPlanner()->Plan(poses)) {
            gotojoints = RCS::Cnc.MoveitPlanner()->GetJtsPlan();
            return gotojoints;
        }
#else
        // assume first one is where were are already
        for(size_t j=1; j< poses.size(); j++) {
            if (RCS::Cnc.MoveitPlanner()->Plan(poses[j-1], poses[j])) {
                std::vector<JointState> intermedjoints;
                intermedjoints = RCS::Cnc.MoveitPlanner()->GetJtsPlan();
                gotojoints.insert(gotojoints.end(), intermedjoints.begin(), intermedjoints.end());
            }
        }
#endif
#ifdef DEBUG
        std::cout << "*******CARTESIAN MOVE TO POSE******\n";
        for (size_t k = 0; k < gotojoints.size(); k++) {
            std::cout << VectorDump<double> (gotojoints[k].position);
        }
#endif
        return gotojoints;

    } else // if (RCS::Cnc.eCartesianMotionPlanner == RCS::CController::WAYPOINT) {
    {
#ifdef MOVEITKIN
        std::vector<double> oldjoints = RCS::Controller.status.currentjoints.position;
        for (size_t i = 0; i < poses.size(); i++) {

            //RCS::Cnc.Kinematics()->SetJointValues(oldjoints);
            std::vector<double> joints = RCS::Cnc.Kinematics()-> IK(poses[i], oldjoints);
#ifdef DEBUG
            std::cout << "GotoPose " << DumpPose(poses[i]).c_str();
            std::cout << "New Joints " << VectorDump<double>(joints).c_str();
#endif
            gotojoints.push_back(EmptyJointState(joints.size()));
            gotojoints.back().position = joints;
            oldjoints = joints;
        }
 #endif
       return gotojoints;
    }
}

int SimpleMotionInterpreter::ParseCommand(RCS::CanonCmd cc) {
 //   IfDebug(Globals.ErrorMessage("SimpleMotionInterpreter::ParseCommand\n"));

    // This approach should debounce multiple commands to same position - e.g., 0->30, 0->30
    JointState currentjoints;
    RCS::Pose currentpose; // = RCS::Controller.status.currentpose;

    //RCS::CanonCmd lastrobotcmd = Cnc.GetLastRobotCommand();
    currentjoints = RCS::Cnc.GetLastJointState(); //  open loop - "not actual"
    currentpose = RCS::Cnc.Kinematics()->FK(currentjoints.position);
#ifdef HEAVYDEBUG
    std::cout << "Current Joints " << VectorDump<double>(currentjoints.position).c_str();
    std::cout << "Current Pose " << DumpPose(currentpose).c_str();
#endif
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SET GRIPPERTS
        if (cc.crclcommand == CanonCmdType::CANON_SET_GRIPPER) {
        RCS::CanonCmd newcc=cc;
        //newcc.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
        //newcc.eepercent=cc.eepercent;
        newcc.status = CanonStatusType::CANON_WAITING;
         Cnc.robotcmds.AddMsgQueue(newcc);            
        }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // MOVE JOINTS
    else if (cc.crclcommand == CanonCmdType::CANON_MOVE_JOINT) {
        rates = IRate(DEFAULT_JOINT_MAX_VEL, DEFAULT_JOINT_MAX_ACCEL, DEFAULT_LOOP_CYCLE);

        JointState joints;
        for (size_t i = 0; i < currentjoints.position.size(); i++) {
            joints.position.push_back(currentjoints.position[i]);
            joints.velocity.push_back(DEFAULT_JOINT_MAX_VEL);
            joints.effort.push_back(DEFAULT_JOINT_MAX_ACCEL);
        }

        std::vector<double> maxvel;
        std::vector<double> maxacc;
        // Check each joint, to see if joint is being actuated, if so, change goal position
        for (size_t i = 0; i < cc.jointnum.size(); i++) {
            size_t n = cc.jointnum[i];
            joints.position[n] = cc.joints.position[n]; // joint numbers already adjusted from CRCL to rcs model
            joints.velocity[n] = cc.joints.velocity[n];
            joints.effort[n] = cc.joints.effort[n];
            maxvel.push_back(cc.joints.velocity[n]);
            maxacc.push_back(cc.joints.effort[n]);
        }
        // not sure what happends if no elements in maxvel or maxacc... - should never happen
        double jointsmaxvel = *(std::min_element(std::begin(maxvel), std::end(maxvel)));
        double jointsmaxacc = *(std::min_element(std::begin(maxacc), std::end(maxacc)));
#ifdef HEAVYDEBUG
        std::cout << "Updated Joints Position " << VectorDump<double>(joints.position).c_str();
        std::cout << "Updated Joints Velocity " << VectorDump<double>(joints.velocity).c_str();
        std::cout << "Updated Joints Effort " << VectorDump<double>(joints.effort).c_str();
#endif
        std::vector<JointState > gotojoints;

        if (RCS::Cnc.eJointMotionPlanner == RCS::CController::WAYPOINT) {
            gotojoints = motioncontrol.computeCoorindatedWaypoints(currentjoints.position, joints.position, 0.001, true);
        } else if (RCS::Cnc.eJointMotionPlanner == RCS::CController::BASIC) {
            TrajectoryMaker maker;
            maker.Rates().CurrentFeedrate() = jointsmaxvel;
            maker.Rates().MaximumAccel() = jointsmaxacc;
            maker.setRates(rates);
            maker.makeJointPositionTrajectory(rates, currentjoints.position, joints.position);
            gotojoints = maker.GetJtsPlan();
        } 
#ifdef MOVEITPLANNER
        else if (RCS::Cnc.eJointMotionPlanner == RCS::CController::MOVEIT) {
            if (!RCS::Cnc.MoveitPlanner()->Plan(joints))
                return -1;
            gotojoints = RCS::Cnc.MoveitPlanner()->GetJtsPlan();
        } 
#endif
        else {
            gotojoints = motioncontrol.computeCoorindatedWaypoints(currentjoints.position, joints.position, 0.001, true);
        }
        if(cc.bCoordinated)
        {
            AddJointCommands(gotojoints);
        }
        else
        {
            // uncoordinated motion - 1st 0 joint, then 1 joint motion, etc.
            // unlikely to crash into itself with this joint motion
            for(size_t k=0; k< currentjoints.position.size(); k++)
            {
                JointState  eachjoint;
                eachjoint.position = currentjoints.position;
                eachjoint.position[k]=joints.position[k];
                TrajectoryMaker maker;
                maker.Rates().CurrentFeedrate() = jointsmaxvel;
                maker.Rates().MaximumAccel() = jointsmaxacc;
                maker.setRates(rates);
                maker.makeJointPositionTrajectory(rates, currentjoints.position, eachjoint.position);
                gotojoints = maker.GetJtsPlan();
                AddJointCommands(gotojoints);
            }
        }
    }////////////////////////////////////////////////////////////////////////////////////////////////
        // STOP MOTION
    else if (cc.crclcommand == CanonCmdType::CANON_STOP_MOTION) {
        std::vector<std::vector<double> > displacements(Cnc.status.currentjoints.velocity.size(), std::vector<double>());
        cc.jointnum.clear();

        // clear motion queue   - we are stopping asap!
        Cnc.robotcmds.ClearMsgQueue();
    }////////////////////////////////////////////////////////////////////////////////////////////////
        // MOVE CARTESIAN
    else if (cc.crclcommand == CanonCmdType::CANON_MOVE_TO) {

        rates = cc.Rates(); // IRate(DEFAULT_CART_MAX_VEL, DEFAULT_CART_MAX_ACCEL, DEFAULT_LOOP_CYCLE);
        RCS::Pose goalpose ;
        tf::poseMsgToTF (cc.finalpose, goalpose);
#ifdef MOVEITKIN
        if (RCS::Cnc.Kinematics()->IsSingular(goalpose, 0.0001)) {
            std::cout << "Is singular pose: " << DumpPose(goalpose).c_str();
        }
#endif
        std::vector<JointState> gotojoints;
        std::cout << "Current Pose " << DumpPose(currentpose).c_str();
        std::vector<RCS::Pose> poses = motioncontrol.computeWaypoints(currentpose, goalpose, 
                0.01, // cc.Rates().CurrentFeedrate() * DEFAULT_LOOP_CYCLE, //0.01, 
                true);
        for (size_t k = 0; k < poses.size(); k++)
            std::cout << "\n\nWaypoint[" << k << "]" << DumpPose(poses[k]).c_str();

        gotojoints = PlanCartesianMotion(poses);
        AddJointCommands(gotojoints);
    } else if (cc.crclcommand == CanonCmdType::CANON_MOVE_THRU) {
        rates = IRate(DEFAULT_CART_MAX_VEL, DEFAULT_CART_MAX_ACCEL, DEFAULT_LOOP_CYCLE);
        std::vector<JointState> gotojoints;
        // FIXME: waypoints must have a point!
        std::vector<RCS::Pose> poses ( sizeof(cc.waypoints)/sizeof(cc.waypoints[0]));
        poses.insert(poses.begin(), currentpose); // add beginning pose -again?
        // and in case interrupted
        gotojoints = PlanCartesianMotion(poses);
        AddJointCommands(gotojoints);
    } else if (cc.crclcommand == CanonCmdType::CANON_DWELL) {
        // wait here or at robot command thread?
        // Could just copy over command to robot
        RCS::CanonCmd newcc;
        newcc.crclcommand = CanonCmdType::CANON_DWELL;
        newcc.status = CanonStatusType::CANON_WAITING;
        newcc.dwell_seconds = cc.dwell_seconds;
        Cnc.robotcmds.AddMsgQueue(newcc);
    }
    return 0;
}
