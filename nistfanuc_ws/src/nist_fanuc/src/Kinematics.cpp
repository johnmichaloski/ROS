

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

static RCS::Pose EMatrix2Pose(Eigen::Matrix4d& m) {
    RCS::Pose pose;
    Eigen::Vector3d trans(m.block<3, 1>(0, 3));
    Eigen::Quaterniond q(m.block<3, 3>(0, 0));
    //pose.setBasis(m.block<3, 3>(0, 0));
    pose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w())); // pose.getRotation());
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
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.0085, 0, -.0041), Eigen::Vector3d(0, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.04191, -.0306, 0), Eigen::Vector3d(1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .00508, .03134), Eigen::Vector3d(3.1415, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(.04843, -.0127, 0), Eigen::Vector3d(-1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .04196, -.0388), Eigen::Vector3d(0, 0, 0)));
    RCS::Pose pose = ComputeFk();
    LOG_DEBUG << "Gripper Offset Pose " << RCS::DumpPoseSimple(pose).c_str();

}

Eigen::Vector3d UrdfVector2EigenVector(const urdf::Vector3 &in) {
    Eigen::Vector3d out;
    out << in.x, in.y, in.z;
    return out;
}

// http://docs.ros.org/jade/api/urdf/html/
//http://docs.ros.org/diamondback/api/urdf/html/classurdf_1_1Pose.html
//http://docs.ros.org/diamondback/api/urdf/html/classurdf_1_1Rotation.html
// http://docs.ros.org/diamondback/api/urdf/html/classurdf_1_1Vector3.html
// Auto in quotes so far

RCS::Pose AutoComputeGripperOffset(urdf::Model& robot_model, std::string prefix) {
#if 0
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.0085, 0, -.0041), Eigen::Vector3d(0, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.04191, -.0306, 0), Eigen::Vector3d(1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .00508, .03134), Eigen::Vector3d(3.1415, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(.04843, -.0127, 0), Eigen::Vector3d(-1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .04196, -.0388), Eigen::Vector3d(0, 0, 0)));
#endif
    AllM.clear();
    std::vector<boost::shared_ptr <const urdf::Joint>> joints;
    std::string grippernames [] = {
        std::string("robotiq_85_base_joint"),
        std::string("robotiq_85_right_knuckle_joint"),
        std::string("robotiq_85_right_finger_joint"),
        std::string("robotiq_85_right_inner_knuckle_joint"),
        std::string("robotiq_85_right_finger_tip_joint")
    };
 //   std::string prefix = "fanuc_";
    
    joints.push_back(robot_model.getJoint(prefix+grippernames [0]));
    joints.push_back(robot_model.getJoint(prefix+grippernames [1]));
    joints.push_back(robot_model.getJoint(prefix+grippernames [2]));
    joints.push_back(robot_model.getJoint(prefix+grippernames [3]));
    joints.push_back(robot_model.getJoint(prefix+grippernames [4]));
    for (size_t i = 0; i < joints.size(); i++) {
        assert(joints[i] != NULL);
        urdf::Vector3 axis = joints[i]->axis;
        urdf::Pose jpose = joints[i]->parent_to_joint_origin_transform;
        urdf::Vector3 position = jpose.position;
        double roll, pitch, yaw;
        jpose.rotation.getRPY(roll, pitch, yaw);
        urdf::Vector3 rpy(roll, pitch, yaw);
        AllM.push_back(ComputeUrdfTransform(0.0,
                UrdfVector2EigenVector(axis),
                UrdfVector2EigenVector(position),
                UrdfVector2EigenVector(rpy)));
    }
    RCS::Pose pose = ComputeFk();
    LOG_DEBUG << "Gripper Offset Pose " << RCS::DumpPoseSimple(pose).c_str();
    return pose;

}

std::vector<double> FastKinematics::FindBoundedSolution(std::vector<std::vector<double>> &solutions,
        std::vector<double> &min,
        std::vector<double> &max) {
    std::vector<size_t> indexes;
    //std::vector<std::vector<double>> dist(solutions.size(), std::vector<double>(solutions[0].size(), 0.0));
    for (size_t i = 0; i < solutions.size(); i++) {
        size_t j;
        for (j = 0; j < solutions[i].size(); j++) {

            LOG_DEBUG << Globals.StrFormat("Val[%d,%d] = %4.2f Min=%4.2f max=%4.2f\n", i, j, solutions[i][j], min[j], max[j]);
            // don't care
            if (0.0 == min[j] && 0.0 == max[j])
                continue;
            // if outside range, break, try next solution
            if (solutions[i][j] < min[j] || (solutions[i][j] > max[j]))
                break;
        }
        if (j == solutions[i].size()) {
            indexes.push_back(i);
            return solutions[i];
        }
        // else no match
    }
    return std::vector<double>(); // no match
}

void FanucLRmate200iD::Configure(int config, size_t size, std::vector<double>& min, std::vector<double> &max) {
    _config = config;
    _size = size;
    min = std::vector<double>(size, -M_PI_2);
    max = std::vector<double>(size, M_PI_2);
    if (config & BASE_FLIP) {
        min[0] = M_PI_2;
        max[0] = M_PI;
        // could be flipped in negative direction
    }
    // Assume shoulder down
    min[1] = 0;
    max[1] = M_PI;
    if (config & SHOULDER_RIGHT) {
        // not handled

    }
    if (config & SHOULDER_DOWN) {
        min[1] = 0;
        max[1] = M_PI;

    }
    if (config & SHOULDER_UP) {
        min[1] = M_PI;
        max[1] = 2. * M_PI;

    }
    if (config & ELBOW_DOWN) {
        min[2] = -M_PI_2;
        max[2] = M_PI_2;
    }
    if (config & ELBOW_UP) {
        min[2] = M_PI_2;
        max[2] = M_PI;
    }
    if (config & FOREARM_DOWN) {
        min[3] = -M_PI_2;
        max[3] = 2. * M_PI_2;
    }
    if (config & FOREARM_UP) {
        min[3] = M_PI_2;
        max[3] = 2. * M_PI;
    }

    if (config & WRIST_NORMAL) {
        min[5] = -M_PI;
        max[5] = M_PI;
    }
    if (config & WRIST_FLIP) {
        min[5] = M_PI;
        max[5] = 2 * M_PI;
        // Could also be negative pi to flip wrist...
    }

    // copy to self?
    _min = min;
    _max = max;
}
