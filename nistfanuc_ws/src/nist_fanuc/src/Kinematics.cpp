
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.

//#pragma message "Compiling " __FILE__ 
#include "Kinematics.h"
#include <eigen_conversions/eigen_msg.h>
#include "RosConversions.h"
#include <iostream>
#include "Conversions.h"
#include "Globals.h"
#include "Debug.h"
#include <boost/format.hpp>
#include "BLogging.h"

bool IKinematics::ParseURDF(std::string xml_string, std::string base_frame) {


   urdf::Model robot_model;
#if 0
    std::string xml_string;

    std::string urdf_xml, full_urdf_xml;
    node_handle.param("urdf_xml", urdf_xml, robot_description);
    node_handle.searchParam(urdf_xml, full_urdf_xml);

    ROS_DEBUG_NAMED("ikfast", "Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string)) {
        ROS_FATAL_NAMED("ikfast", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    node_handle.param(full_urdf_xml, xml_string, std::string());
#endif
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("ikfast", "Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(getTipFrame()));
    while (link->name != base_frame){ // && joint_names.size() <= num_joints_) {
        ROS_DEBUG_NAMED("ikfast", "Link %s", link->name.c_str());
        link_names.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                ROS_DEBUG_STREAM_NAMED("ikfast", "Adding joint " << joint->name);

                joint_names.push_back(joint->name);
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
            ROS_WARN_NAMED("ikfast", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(link_names.begin(), link_names.end());
    std::reverse(joint_names.begin(), joint_names.end());
    std::reverse(joint_min.begin(), joint_min.end());
    std::reverse(joint_max.begin(), joint_max.end());
    std::reverse(joint_has_limits.begin(), joint_has_limits.end());

//    for (size_t i = 0; i < num_joints; ++i)
//        ROS_DEBUG_STREAM_NAMED("ikfast", joint_names[i] << " " << joint_min[i] << " " << joint_max[i] << " " << joint_has_limits[i]);

    return true;
}

// Could find basics explained here: http://aeswiki.datasys.swri.edu/rositraining/Exercises/3.6

// https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/motion_planning_api_tutorial.cpp
// http://moveit.ros.org/wiki/Motion_Planning/C%2B%2B_API
// http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/motion_planning_api_tutorial.html
// http://docs.ros.org/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#aa28a400ac63222f07598c53c685d7144



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


#if 0

//////////////////////////////////////////////////////////////////
#include <algorithm>

void MotomanSia20d::fillFreeParams(int count, int *array) {
    free_params_.clear();
    for (int i = 0; i < count; ++i) free_params_.push_back(array[i]);
}

bool MotomanSia20d::initialize(const std::string &robot_description,
        const std::string& group_name,
        const std::string& base_name,
        const std::string& tip_name,
        double search_discretization) {

    ros::NodeHandle node_handle("~/" + group_name);

    std::string robot;
    node_handle.param("robot", robot, std::string());

    // IKFast56/61
    fillFreeParams(GetNumFreeParameters(), GetFreeParameters());
    num_joints_ = GetNumJoints();

#if 0
    if (free_params_.size() > 1) {
        ROS_FATAL("Only one free joint parameter supported!");
        return false;
    } else if (free_params_.size() == 1) {
        redundant_joint_indices_.clear();
        redundant_joint_indices_.push_back(free_params_[0]);
        KinematicsBase::setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);
    }
#endif
    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml, full_urdf_xml;
    node_handle.param("urdf_xml", urdf_xml, robot_description);
    node_handle.searchParam(urdf_xml, full_urdf_xml);

    ROS_DEBUG_NAMED("ikfast", "Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string)) {
        ROS_FATAL_NAMED("ikfast", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    node_handle.param(full_urdf_xml, xml_string, std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("ikfast", "Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(getTipFrame()));
    while (link->name != base_frame_ && joint_names_.size() <= num_joints_) {
        ROS_DEBUG_NAMED("ikfast", "Link %s", link->name.c_str());
        link_names_.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                ROS_DEBUG_STREAM_NAMED("ikfast", "Adding joint " << joint->name);

                joint_names_.push_back(joint->name);
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
                    joint_has_limits_vector_.push_back(true);
                    joint_min_vector_.push_back(lower);
                    joint_max_vector_.push_back(upper);
                } else {
                    joint_has_limits_vector_.push_back(false);
                    joint_min_vector_.push_back(-M_PI);
                    joint_max_vector_.push_back(M_PI);
                }
            }
        } else {
            ROS_WARN_NAMED("ikfast", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    if (joint_names_.size() != num_joints_) {
        ROS_FATAL_STREAM_NAMED("ikfast", "Joint numbers mismatch: URDF has " << joint_names_.size() << " and IKFast has " << num_joints_);
        return false;
    }

    std::reverse(link_names_.begin(), link_names_.end());
    std::reverse(joint_names_.begin(), joint_names_.end());
    std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
    std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());
    std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

    for (size_t i = 0; i < num_joints_; ++i)
        ROS_DEBUG_STREAM_NAMED("ikfast", joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " " << joint_has_limits_vector_[i]);

    active_ = true;
    return true;
}

double MotomanSia20d::harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
    double dist_sqr = 0;
    std::vector<double> ss = ik_seed_state;
    for (size_t i = 0; i < ik_seed_state.size(); ++i) {
        while (ss[i] > 2 * M_PI) {
            ss[i] -= 2 * M_PI;
        }
        while (ss[i] < 2 * M_PI) {
            ss[i] += 2 * M_PI;
        }
        while (solution[i] > 2 * M_PI) {
            solution[i] -= 2 * M_PI;
        }
        while (solution[i] < 2 * M_PI) {
            solution[i] += 2 * M_PI;
        }
        dist_sqr += fabs(ik_seed_state[i] - solution[i]);
    }
    return dist_sqr;
}

void MotomanSia20d::getClosestSolution(const IkSolutionList<double> &solutions, const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
    double mindist = DBL_MAX;
    int minindex = -1;
    std::vector<double> sol;

    // IKFast56/61
    for (size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        getSolution(solutions, i, sol);
        double dist = harmonize(ik_seed_state, sol);
        ROS_INFO_STREAM_NAMED("ikfast", "Dist " << i << " dist " << dist);
        //std::cout << "dist[" << i << "]= " << dist << std::endl;
        if (minindex == -1 || dist < mindist) {
            minindex = i;
            mindist = dist;
        }
    }
    if (minindex >= 0) {
        getSolution(solutions, minindex, solution);
        harmonize(ik_seed_state, solution);
    }
}
#endif