// RCS.cpp

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
#include "RCS.h"
#include "Globals.h"
#include "Controller.h"
//#include "crcl.h"
namespace RCS {

    std::string DumpPose(RCS::Pose & pose) {
        std::stringstream s;

        s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z() << std::endl;
        double roll, pitch, yaw;
        getRPY(pose, roll, pitch, yaw);
        s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw) << std::endl;
        s << "Quaternion = " << pose.getRotation().x() << ":" << pose.getRotation().y() << ":" << pose.getRotation().z() << ":" << pose.getRotation().w() << std::endl;
        //s << Crcl::DumpRotationAsCrcl(pose)<< std::endl;
        return s.str();
    }

    void getRPY(const RCS::Pose pose, double &roll, double &pitch, double &yaw) {
        tf::Matrix3x3 rot = pose.getBasis();
        rot.getRPY(roll, pitch, yaw);
    }

    unsigned long long CanonCmd::_cmdid = 0;

    void CanonWorldModel::Init() {
        _cycleTime = DEFAULT_LOOP_CYCLE;

        maxTransAccel = DEFAULT_CART_MAX_ACCEL;
        maxTransVel = DEFAULT_CART_MAX_VEL;
        maxRotAccel = DEFAULT_CART_MAX_ACCEL;
        maxRotVel = DEFAULT_CART_MAX_VEL;
        maxJointAccel = DEFAULT_JOINT_MAX_ACCEL;
        maxJointVel = DEFAULT_JOINT_MAX_VEL;

        //for (size_t i = 0; i < RCS::Cnc.robot_model.GetMovingJoints(); i++) {
        for (size_t i = 0; i < RCS::Cnc.Kinematics()->NumJoints(); i++) {
            currentjoints.position.push_back(0.0);
            currentjoints.velocity.push_back(0.0);
        }
    }
    // -----------------------------------------------------------------

    void CanonCmd::Init() {
        crclcommand = CanonCmdType::CANON_NOOP;
        status = CanonStatusType::CANON_WAITING;
        accprofile = CanonAccProfile::MS_IS_UNSET;
        type = TrajPointType::GOAL;
        stoptype = CanonStopMotionType::UNSET;
        bCoordinated = false;
        bStraight = false;

        // In theory these can change
        absTransAcc = DEFAULT_CART_MAX_ACCEL;
        absTransSpeed = DEFAULT_CART_MAX_VEL;
        absRotAcc = DEFAULT_CART_MAX_ACCEL;
        absRotSpeed = DEFAULT_CART_MAX_VEL;
        absJointAcc = DEFAULT_JOINT_MAX_ACCEL;
        absJointSpeed = DEFAULT_JOINT_MAX_VEL;
    }

    bool CanonCmd::IsMotionCmd() {
        static int motions[] = {CanonCmdType::CANON_MOVE_JOINT,
            CanonCmdType::CANON_MOVE_TO,
            CanonCmdType::CANON_MOVE_THRU,
            CanonCmdType::CANON_SET_GRIPPER,
            CanonCmdType::CANON_STOP_MOTION};
        int * it = std::find(motions, motions +sizeof(motions)/sizeof(int), crclcommand);
        if (it != motions + 5)
            return true;
        return false;
    }
};
