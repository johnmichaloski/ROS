

#include "Kinematics.h"
#include <eigen_conversions/eigen_msg.h>
#include "RosConversions.h"
#include <iostream>
#include "Conversions.h"
#include "Globals.h"
#include "Debug.h"

// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.

// Could find basics explained here: http://aeswiki.datasys.swri.edu/rositraining/Exercises/3.6

// https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/motion_planning_api_tutorial.cpp
// http://moveit.ros.org/wiki/Motion_Planning/C%2B%2B_API
// http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/motion_planning_api_tutorial.html
// http://docs.ros.org/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#aa28a400ac63222f07598c53c685d7144


#include <boost/format.hpp>
#include "BLogging.h"


/**
 * THis code will produce the pose offset given the gripper definition. The axis and origin (xyz rpy) 
 * are hard coded at this point - failed attempt to find these fields at this point.
 */
std::string DumpEPosition(const Eigen::Vector3d & v) {
    std::stringstream s;
    for (int j = 0; j < v.size(); j++) {
        s << boost::format("%8.5f") % v(0) << boost::format("%8.5f") % v(1) << boost::format("%8.5f") % v(2) << "\n";
    }
    return s.str();
}

struct UrdfJoint {
    std::string name;
    Eigen::Vector3d axis;
    Eigen::Vector3d xyzorigin;
    Eigen::Vector3d rpyorigin;

    std::string DumpUrdfJoint() {
        const UrdfJoint & j(*this);
        std::stringstream s;
        s << " Axis = " << DumpEPosition(j.axis).c_str() << std::endl;
        s << " XYZ Origin = " << DumpEPosition(j.xyzorigin).c_str() << std::endl;
        s << " RPY Origin = " << DumpEPosition(j.rpyorigin).c_str() << std::endl;
        return s.str();
    }
};

static std::vector<Eigen::Matrix4d> A0;
static std::vector<Eigen::Matrix4d> AllM;

Eigen::Matrix4d ComputeUrdfTransform(double angle,
        Eigen::Vector3d axis,
        Eigen::Vector3d origin,
        Eigen::Vector3d rotation) {
    Eigen::Matrix3d t33;
    Eigen::Matrix4d m1 = Eigen::Matrix4d::Identity(); // Create4x4IdentityMatrix();
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();

    Eigen::Vector3d unit = axis.array().abs();
    t33 = Eigen::AngleAxisd(axis.sum() * angle, unit);
    m1.block<3, 3>(0, 0) = t33;
    tmp.block<3, 1>(0, 3) = origin;
    return tmp * m1; // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
}
static RCS::Pose EMatrix2Pose(Eigen::Matrix4d& m)
{
    RCS::Pose pose;
    Eigen::Vector3d trans(m.block<3, 1>(0, 3));
    Eigen::Quaterniond q(m.block<3, 3>(0, 0));
    //pose.setBasis(m.block<3, 3>(0, 0));
    pose.setRotation(tf::Quaternion(q.x(),q.y(), q.z(), q.w()));// pose.getRotation());
    pose.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
 //   tf::vectorEigenToTF(trans, pose.getOrigin());
    return pose;
}

static void quaternionEigenToTF(const Eigen::Quaterniond& e, tf::Quaternion& t) {
    t[0] = e.x();
    t[1] = e.y();
    t[2] = e.z();
    t[3] = e.w();
}
 
static RCS::Pose ComputeFk()// std::vector<double> thetas)
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    A0.clear();

    //	for ( int i = 0; i< thetas.size( ); i++ )
    //	{
    //		AllM.push_back(ComputeUrdfTransform(thetas[i], jointspec[i].axis, jointspec[i].xyzorigin, jointspec[i].rpyorigin));
    //	}

    for (size_t i = 0; i < AllM.size(); i++) {
        t = t * AllM[i];
        A0.push_back(t);
    }

    return EMatrix2Pose(t);
}


/**
 * robotiq_85_base_joint                axis=1,0,0 rpy="0 0 0" xyz=".0085 0 -.0041"
 * robotiq_85_right_knuckle_joint       axis=1,0,0 rpy="1.5707 -1.5707 0" xyz=".04191 -.0306 0"
 * robotiq_85_right_finger_joint 	axis=1,0,0 rpy="3.1415 0 0" xyz="0 .00508 .03134"
 * robotiq_85_right_inner_knuckle_joint axis=1,0,0 rpy="-1.5707 -1.5707 0" xyz=".04843 -.0127 0"
 * robotiq_85_right_finger_tip_joint 	axis=1,0,0 rpy="0 0 0" xyz="0 .04196 -.0388"
 * @return 
 */
RCS::Pose ComputeGripperOffset() {
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.0085 ,0 ,-.0041), Eigen::Vector3d(0, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.04191, -.0306, 0), Eigen::Vector3d(1.5707, - 1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .00508, .03134), Eigen::Vector3d(3.1415, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(.04843 ,- .0127, 0), Eigen::Vector3d(-1.5707, - 1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0 ,.04196, - .0388), Eigen::Vector3d(0, 0, 0)));
    RCS::Pose  pose = ComputeFk();
    LOG_DEBUG << "Gripper Offset Pose " << RCS::DumpPoseSimple(pose).c_str();
    
} 