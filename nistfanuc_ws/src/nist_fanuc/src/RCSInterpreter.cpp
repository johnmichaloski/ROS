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
#include "trajectoryMaker.h"
#include "Conversions.h"
#include "Debug.h"
#include "gomotion/gomove.h"
using namespace RCS;
using namespace sensor_msgs;
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
BangBangInterpreter::BangBangInterpreter(boost::shared_ptr<RCS::CController> nc,
        IKinematicsSharedPtr k)
//        NearestJointsLookup &hints)
: _nc(nc), _kinematics(k){ // , _hints(hints) {
}

void BangBangInterpreter::SetRange(std::vector<double> minrange, std::vector<double> maxrange) {
    this->minrange = minrange;
    this->maxrange = maxrange;
}

int BangBangInterpreter::ParseCommand(RCS::CanonCmd cmd, RCS::CanonCmd &outcmd) {

    if (cmd.crclcommand == CanonCmdType::CANON_MOVE_JOINT) {
        // Move immediately to joint value - Fill in other joints with current values
        cmd.joints = _kinematics->UpdateJointState(cmd.jointnum, _nc->status.currentjoints, cmd.joints);
    } else if (cmd.crclcommand == CanonCmdType::CANON_MOVE_TO) {
        // FIXME: need to subtract off tool offset from tcp
        RCS::Pose finalpose = Conversion::GeomMsgPose2RcsPose(cmd.finalpose);
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
        LOG_DEBUG << "IK Joints " << VectorDump<double>(  cmd.joints.position).c_str();
        cmd.joints.name = _kinematics->JointNames();
        cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
         
    }
    outcmd=cmd;
   //_nc->robotcmds.AddMsgQueue(cmd);

    return CanonStatusType::CANON_DONE;
}
