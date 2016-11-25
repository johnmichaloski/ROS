
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.

//#pragma message "Compiling " __FILE__ 
#include "Kinematics.h"

#include <iostream>

#include <boost/format.hpp>


#include "RosConversions.h"
#include "Conversions.h"
#include "Globals.h"
#include "Debug.h"
#include "Boost.h"

using namespace Conversion;

bool IKinematics::ParseURDF(std::string xml_string, std::string base_frame) {
    urdf::Model robot_model;
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("nc", "Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(getTipLink()));
    while (link->name != base_frame){ // && joint_names.size() <= num_joints_) {
        ROS_DEBUG_NAMED("nc", "Link %s", link->name.c_str());
        link_names.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                ROS_DEBUG_STREAM_NAMED("nc", "Adding joint " << joint->name);

                joint_names.push_back(joint->name);
                axis.push_back(Convert<urdf::Vector3,Eigen::Vector3d>(joint->axis));
                xyzorigin.push_back(Convert<urdf::Vector3,Eigen::Vector3d>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY (roll, pitch, yaw);
                rpyorigin.push_back(Eigen::Vector3d(roll,pitch,yaw));
                
                float lower, upper;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit;
                        upper = joint->safety->soft_upper_limit;
                    } else {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    hasLimits = 0;
                }
                if (hasLimits) {
                    joint_has_limits.push_back(true);
                    joint_min.push_back(lower);
                    joint_max.push_back(upper);
                } else {
                    joint_has_limits.push_back(false);
                    joint_min.push_back(-M_PI);
                    joint_max.push_back(M_PI);
                }
            }
        } else {
            ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(link_names.begin(), link_names.end());
    std::reverse(joint_names.begin(), joint_names.end());
    std::reverse(joint_min.begin(), joint_min.end());
    std::reverse(joint_max.begin(), joint_max.end());
    std::reverse(joint_has_limits.begin(), joint_has_limits.end());
    std::reverse(axis.begin(), axis.end());
    std::reverse(xyzorigin.begin(), xyzorigin.end());
    std::reverse(rpyorigin.begin(), rpyorigin.end());

//    for (size_t i = 0; i < num_joints; ++i)
//        ROS_DEBUG_STREAM_NAMED("nc", joint_names[i] << " " << joint_min[i] << " " << joint_max[i] << " " << joint_has_limits[i]);

    LOG_DEBUG <<  DumpUrdfJoint().c_str();

    return true;
}

std::string IKinematics::DumpUrdfJoint() {
    std::stringstream s;
    for (int i = 0; i < joint_names.size(); i++) {
        s << "Joint = " << joint_names[i].c_str()<< std::endl;
        s << " Axis = " << RCS::DumpEVector(axis[i]).c_str();
        s << " XYZ Origin = " << RCS::DumpEVector(xyzorigin[i]).c_str();
        s << " RPY Origin = " << RCS::DumpEVector(rpyorigin[i]).c_str();
    }
    return s.str();
}

Eigen::Matrix4d IKinematics::ComputeUrdfTransform(double angle,
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

std::vector<tf::Pose> IKinematics::ComputeAllFk(std::vector<double> thetas)
{
    // Using URDF will compute FK
    std::vector<Eigen::Matrix4d> AllM;
    std::vector<Eigen::Matrix4d> A0;
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    std::vector<tf::Pose> jointposes;
    A0.clear();

    for (int i = 0; i < joint_names.size(); i++) {
        AllM.push_back(ComputeUrdfTransform(thetas[i], axis[i], xyzorigin[i], rpyorigin[i]));
    }

    for (size_t i = 0; i < AllM.size(); i++) {
        t = t * AllM[i];
        A0.push_back(t);
        jointposes.push_back(Convert<Eigen::Matrix4d, tf::Pose>(t));
    }

    return jointposes;
}
#if 0
Eigen::Matrix4d ComputeUrdfTransform(double angle, Eigen::Vector3d axis, 
        Eigen::Vector3d origin, 
        Eigen::Vector3d rotation)
{
	Eigen::Matrix3d t33;
        Eigen::Matrix4d m1 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        
	Eigen::Vector3d unit = axis.array().abs();
	t33 = Eigen::AngleAxisd(axis.sum() * angle, unit );
	m1.block<3,3>(0,0) =t33;
	tmp.block<3,1>(0,3) = origin;
	return  tmp * m1 ;  // i dont understand why this matrix multiply order works!
}
#endif

#if 0
// Could find basics explained here: http://aeswiki.datasys.swri.edu/rositraining/Exercises/3.6

// https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/motion_planning_api_tutorial.cpp
// http://moveit.ros.org/wiki/Motion_Planning/C%2B%2B_API
// http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/motion_planning_api_tutorial.html
// http://docs.ros.org/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#aa28a400ac63222f07598c53c685d7144

// This sorta computes the pose offset of the robotiq gripper. Final  X length is not correct.

/**
 * robotiq_85_base_joint                axis=1,0,0 rpy="0 0 0" xyz=".0085 0 -.0041"
 * robotiq_85_right_knuckle_joint       axis=1,0,0 rpy="1.5707 -1.5707 0" xyz=".04191 -.0306 0"
 * robotiq_85_right_finger_joint 	axis=1,0,0 rpy="3.1415 0 0" xyz="0 .00508 .03134"
 * robotiq_85_right_inner_knuckle_joint axis=1,0,0 rpy="-1.5707 -1.5707 0" xyz=".04843 -.0127 0"
 * robotiq_85_right_finger_tip_joint 	axis=1,0,0 rpy="0 0 0" xyz="0 .04196 -.0388"
 * @return 
 */
RCS::Pose ComputeGripperOffset() {
    std::vector<Eigen::Matrix4d> AllM;
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
    std::vector<Eigen::Matrix4d> AllM;
    std::vector<boost::shared_ptr <const urdf::Joint>> joints;
    std::string grippernames [] = {
        std::string("robotiq_85_base_joint"),
        std::string("robotiq_85_right_knuckle_joint"),
        std::string("robotiq_85_right_finger_joint"),
        std::string("robotiq_85_right_inner_knuckle_joint"),
        std::string("robotiq_85_right_finger_tip_joint")
    };
    //   std::string prefix = "fanuc_";

    joints.push_back(robot_model.getJoint(prefix + grippernames [0]));
    joints.push_back(robot_model.getJoint(prefix + grippernames [1]));
    joints.push_back(robot_model.getJoint(prefix + grippernames [2]));
    joints.push_back(robot_model.getJoint(prefix + grippernames [3]));
    joints.push_back(robot_model.getJoint(prefix + grippernames [4]));
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

#endif

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
