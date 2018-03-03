
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.

//#pragma message "Compiling " __FILE__ 
#include "Kinematics.h"

#include <iostream>
#include <boost/format.hpp>


#include "Conversions.h"
#include "Globals.h"
#include "Debug.h"
#include "Boost.h"

using namespace Conversion;
double IKinematics::_testspacing;
double IKinematics::_testoffset;
double IKinematics::_testepsilon;

void IKinematics::ENormalize(double min, double max) {
    joint_emax = joint_max;
    joint_emin = joint_min;

    double range = (fabs(min) + fabs(max));
    for (size_t i = 0; i < joint_emin.size(); i++) {
        double jtrange = (fabs(joint_emin[i]) + fabs(joint_emax[i]));
        if (jtrange > range) {
            joint_emin[i] = min;
            joint_emax[i] = max;
        }
#if 0
        if (joint_emin[i] < min)
            joint_emin[i] = min;
        if (joint_emax[i] > max)
            joint_emax[i] = max;
#endif
    }
}

bool IKinematics::IncrementExercise(std::vector<double>& jts) {
    size_t i = 0;
    jts[i] += _testspacing * (joint_emax[i] - joint_emin[i]) / 2.0;
    size_t j = i;
    while (j < num_joints - 1 && jts[j] >= joint_emax[j]) {
        jts[j + 1] += _testspacing * (joint_emax[j + 1] - joint_emin[j + 1]) / 2.0;
        jts[j] = joint_emin[j] + _testoffset;
        j++;
    }

    if (jts[num_joints - 1] >= joint_emax[num_joints - 1])
        return true;
    return false;
}

bool IKinematics::ParseURDF(std::string xml_string, std::string base_frame) {
    urdf::Model robot_model;
    robot_model.initString(xml_string);

    //ROS_DEBUG_STREAM_NAMED("nc", "Reading joints and links from URDF");

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(getTipLink()));
    while (link->name != base_frame) { // && joint_names.size() <= num_joints_) {
        //ROS_DEBUG_NAMED("nc", "Link %s", link->name.c_str());
        link_names.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                //ROS_DEBUG_STREAM_NAMED("nc", "Adding joint " << joint->name);

                joint_names.push_back(joint->name);
                axis.push_back(Convert<urdf::Vector3, Eigen::Vector3d>(joint->axis));
                xyzorigin.push_back(Convert<urdf::Vector3, Eigen::Vector3d>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                rpyorigin.push_back(Eigen::Vector3d(roll, pitch, yaw));

                float lower, upper, maxvel = 0.0, maxeffort = 0.0;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    maxvel = joint->limits->velocity;
                    maxeffort = joint->limits->effort;
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
                joint_effort.push_back(maxeffort);
                joint_velmax.push_back(maxvel);
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
    std::reverse(joint_effort.begin(), joint_effort.end());
    std::reverse(joint_velmax.begin(), joint_velmax.end());

    return true;
}

// FIXME: <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>

std::string IKinematics::DumpUrdfJoint() {
    std::stringstream s;
    for (int i = 0; i < joint_names.size(); i++) {
        s << "Joint       = " << joint_names[i].c_str() << std::endl;
        s << " Axis       = " << RCS::DumpEVector(axis[i]).c_str();
        s << " XYZ Origin = " << RCS::DumpEVector(xyzorigin[i]).c_str();
        s << " RPY Origin = " << RCS::DumpEVector(rpyorigin[i]).c_str();
        s << " Has Limits = " << joint_has_limits[i] << std::endl;
        s << " Min Pos    = " << joint_min[i] << std::endl;
        ;
        s << " Max Pos    = " << joint_max[i] << std::endl;
        ;
        s << " Max Vel    = " << joint_velmax[i] << std::endl;
        ;
        s << " Max Effort = " << joint_effort[i] << std::endl;
        ;
    }
    return s.str();
}
std::string IKinematics::DumpTransformMatrices() {
    // Fixme: row versus column major :( May not matter since eigen rectifies when accessing
    std::string dump;
    for (int i = 0; i < joint_names.size(); i++) {
        Eigen::Matrix4d m = ComputeUrdfTransform(M_PI_2, axis[i], xyzorigin[i], rpyorigin[i]);
        dump += Globals.StrFormat("%dT%d=\n", i, i + 1);

        dump += RCS::Dump4x4Matrix(m);
    }
    return dump;
}

Eigen::Matrix4d IKinematics::ComputeUrdfTransform(double angle,
        Eigen::Vector3d axis,
        Eigen::Vector3d origin,
        Eigen::Vector3d rotation) {
    Eigen::Matrix3d t33,yaw,pitch, roll;
    Eigen::Matrix4d m1 = Eigen::Matrix4d::Identity(); // Create4x4IdentityMatrix();
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();

    Eigen::Vector3d unit = axis.array().abs();
    t33 = Eigen::AngleAxisd(axis.sum() * angle, unit);
    yaw = Eigen::AngleAxisd(rotation(0), Eigen::Vector3d(1,0,0));
    pitch = Eigen::AngleAxisd(rotation(1), Eigen::Vector3d(0,1,0));
    roll = Eigen::AngleAxisd(rotation(2), Eigen::Vector3d(0,0,1));
    m1.block<3, 3>(0, 0) =  yaw * pitch *  roll * t33;
    tmp.block<3, 1>(0, 3) = origin;
    return tmp * m1; // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
}


std::vector<tf::Pose> IKinematics::ComputeAllFk(std::vector<double> thetas) {
    // Using URDF will compute FK
    std::vector<Eigen::Matrix4d> AllM;
    std::vector<Eigen::Matrix4d> A0;
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    std::vector<tf::Pose> jointposes;
    A0.clear();

    for (int i = 0; i < axis.size(); i++) {
        AllM.push_back(ComputeUrdfTransform(thetas[i], axis[i], xyzorigin[i], rpyorigin[i]));
    }

    for (size_t i = 0; i < axis.size(); i++) {
        t = t * AllM[i];
        A0.push_back(t);
        jointposes.push_back(Convert<Eigen::Matrix4d, tf::Pose>(t));
    }

    return jointposes;
}

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
tf::Pose ComputeGripperOffset() {
    std::vector<Eigen::Matrix4d> AllM;
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.0085, 0, -.0041), Eigen::Vector3d(0, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.04191, -.0306, 0), Eigen::Vector3d(1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .00508, .03134), Eigen::Vector3d(3.1415, 0, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(.04843, -.0127, 0), Eigen::Vector3d(-1.5707, -1.5707, 0)));
    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .04196, -.0388), Eigen::Vector3d(0, 0, 0)));
    tf::Pose pose = ComputeFk();
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

tf::Pose AutoComputeGripperOffset(urdf::Model& robot_model, std::string prefix) {
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
    tf::Pose pose = ComputeFk();
    LOG_DEBUG << "Gripper Offset Pose " << RCS::DumpPoseSimple(pose).c_str();
    return pose;

}

#endif

void FanucLRmate200iD::Configure(int config) {
    _config = config;
    this->bConfig = true;
    _min = std::vector<double>(_size, -M_PI_2);
    _max = std::vector<double>(_size, M_PI_2);
    if (config & BASE_FLIP) {
        _min[0] = -1.6;
        _max[0] = 1.6;
        // could be flipped in negative direction
    }
    // Assume shoulder down
    if (config & SHOULDER_RIGHT) {
        // not handled
        assert(0);

    }
    if (config & SHOULDER_DOWN) {
        _min[1] = 1.53;
        _max[1] = 2.53;

    }
    if (config & SHOULDER_UP) {
        _min[1] = -1.75;
        _max[1] = 1.53;

    }
    if (config & ELBOW_DOWN) {
        _min[2] = -M_PI_2;
        _max[2] = 0.0;
    }
    if (config & ELBOW_UP) {
        _min[2] = 1.70;
        _max[2] = 4.0;
    }
    if (config & FOREARM_DOWN) {
        _min[3] = -M_PI_2;
        _max[3] = 2. * M_PI_2;
    }
    if (config & FOREARM_UP) {
        _min[3] = -.8;
        _max[3] = .8;
    }

    if (config & WRIST_NORMAL) {
        _min[6] = -M_PI_2;
        _max[6] = M_PI_2;
    }
    if (config & WRIST_FLIP) {
        // unclear when outside normal range
    }
}

std::vector<double> FanucLRmate200iD::Seed(int config = -1) {
    if (config != -1)
        Configure(config);
    if (bConfig != true)
        throw std::runtime_error("Not configured");

    std::vector<double> seed;
    double avg;
    for (size_t i = 0; i < _size; i++) {
        avg = _max[i] + _min[i] / 2.0;
        seed.push_back(avg);
    }
    return seed;
}

#include <gokin/gokin.h>
#include "Controller.h"
MotomanSia20dGoKin::MotomanSia20dGoKin(boost::shared_ptr<RCS::CController> nc) {
    _nc = nc;
    _pGoKin = boost::shared_ptr<gomotion::GoKin>(new gomotion::GoKin());
    _pGoKin->SetLengthUnitsPerMeter(1.0);
}

tf::Pose MotomanSia20dGoKin::FK(std::vector<double> joints) {
    return _pGoKin->fwd(joints);
}

size_t MotomanSia20dGoKin::AllIK(tf::Pose & pose,
        std::vector<std::vector<double>> &joints) {
    assert(0);
    return 0;
}

std::vector<double> MotomanSia20dGoKin::IK(tf::Pose pose,
        std::vector<double> oldjoints) {
    return _pGoKin->inv(pose,oldjoints);
}

bool MotomanSia20dGoKin::IsSingular(tf::Pose pose, double threshold) {
    return false;
}

void MotomanSia20dGoKin::Init(ros::NodeHandle & nh) {
    IKinematics::Init(nh);

    std::vector<std::vector<double> > params;
    for (size_t i = 0; i < axis.size(); i++) {
        std::vector<double> d;
        std::vector<double> e = Convert<Eigen::Vector3d, std::vector<double>>(xyzorigin[i]);
        d.insert(d.end(), e.begin(), e.end());
        e = Convert<Eigen::Vector3d, std::vector<double>>(rpyorigin[i]);
        d.insert(d.end(), e.begin(), e.end());
        e = Convert<Eigen::Vector3d, std::vector<double>>(axis[i]);
        d.insert(d.end(), e.begin(), e.end()); //       d.insert(d.end(), rpyorigin.begin(), rpyorigin.end());
        params.push_back(d);
    }
    // 0 0 180 0 0 -180 0 0 -1
    std::vector<double>d = ToVector<double>(9,0.,0.,0.18, 0.,0.,(double)-M_PI_2,0.,0.,-1.);
    //params.push_back(d);
    _pGoKin->SetParams(params);
}
#if 0

#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>

// http://www.orocos.org/kdl/examples
// http://docs.ros.org/jade/api/tf_conversions/html/c++/tf__kdl_8cpp_source.html
void poseTFToKDL(const tf::Pose& t, KDL::Frame& k) {
    for (unsigned int i = 0; i < 3; ++i)
        k.p[i] = t.getOrigin()[i];
    for (unsigned int i = 0; i < 9; ++i)
        k.M.data[i] = t.getBasis()[i / 3][i % 3];
}

void poseKDLToTF(const KDL::Frame& k, tf::Pose& t) {
    t.setOrigin(tf::Vector3(k.p[0], k.p[1], k.p[2]));
    t.setBasis(tf::Matrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
            k.M.data[3], k.M.data[4], k.M.data[5],
            k.M.data[6], k.M.data[7], k.M.data[8]));
}

MotomanSia20dTrak_IK::MotomanSia20dTrak_IK(boost::shared_ptr<RCS::CController> nc) {
    _nc = nc;
}
using namespace KDL;
//http://docs.ros.org/kinetic/api/kdl_parser/html/kdl__parser_8cpp_source.html
//motoman_joint_s = 0.0, 0.0, 0.41,  0.0,-0.0, 0.0,  0.0, 0.0, 1.0
//motoman_joint_l = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0, 1.0, 0.0
//motoman_joint_e = 0.0, 0.0, 0.49,  0.0,-0.0, 0.0,  0.0, 0.0, 1.0
//motoman_joint_u = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0,-1.0, 0.0 
//motoman_joint_r = 0.0, 0.0, 0.42,  0.0,-0.0, 0.0,  0.0, 0.0,-1.0
//motoman_joint_b = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0,-1.0, 0.0
//motoman_joint_t = 0.0, 0.0, 0.18,  0.0,-0.0, 0.0,  0.0, 0.0,-1.0
tf::Pose MotomanSia20dTrak_IK::FK(std::vector<double> joints) {
    KDL::Chain chain;
    _pTRAC_IK->getKDLChain(chain);
 
#if 0
   KDL::Tree tree;
   if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        throw std::runtime_error("Exception: robot_model.treeFromString(xml,tree)");
    }
    if (!tree.getChain(_rootlinkname, "motoman_robotiq_85_adapter_link", chain)) {
        ROS_ERROR("Could not initialize chain object");
        throw std::runtime_error("Exception: tree.getChain)");
    }
#endif
#if 0
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,.41 ), Vector(0,0,1) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.41 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,0.0 ), Vector(0,1,0) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,0.0 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,.49 ), Vector(0,0,1) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.49 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,0.0 ), Vector(0,-1,0) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.41 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,.42 ), Vector(0,0,-1) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.42 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,0.0 ), Vector(0,-1,0) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,0.0 ))));
     
     chain.addSegment(Segment(Joint(Vector(0.0,0.0,.18 ), Vector(0,0,-1) , Joint::RotAxis ),
     KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.18 ))));
     
#endif
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    unsigned int nj = joints.size();
    KDL::JntArray injoints = KDL::JntArray(nj);

    // Assign some values to the joint positions
    for (unsigned int i = 0; i < nj; i++) {
        injoints(i) = (double) joints[i];
    }
    // Create the frame that will contain the results
    KDL::Frame cartpos;
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(injoints, cartpos);
    if (kinematics_status < 0) {
        std::cerr << "Error: could not calculate forward kinematics :(";
    }

    tf::Pose pose;
    poseKDLToTF(cartpos, pose);
    return pose;
}

size_t MotomanSia20dTrak_IK::AllIK(tf::Pose & pose,
        std::vector<std::vector<double>> &joints) {
    assert(0);
    return 0;
}

std::vector<double> MotomanSia20dTrak_IK::IK(tf::Pose pose,
        std::vector<double> oldjoints) {
    // Create joint array
    KDL::Frame kdl_frame;
    poseTFToKDL(pose,kdl_frame);
    unsigned int nj = oldjoints.size();
    KDL::JntArray injoints = KDL::JntArray(nj);

    // Assign some values to the joint positions
    for (unsigned int i = 0; i < nj; i++) {
        injoints(i) = (double) oldjoints[i];
    }
    KDL::JntArray outjoints = KDL::JntArray(nj);
    _pTRAC_IK->CartToJnt(injoints, kdl_frame, outjoints);
    std::vector<double> joints(nj, 0.0);
    for (unsigned int i = 0; i < nj; i++) {
        joints[i] = (double) outjoints(i);
    }
    return joints;
}

bool MotomanSia20dTrak_IK::IsSingular(tf::Pose pose, double threshold) {
    return false;
}

void MotomanSia20dTrak_IK::Init(ros::NodeHandle & nh) {
    IKinematics::Init(nh);
    try{
    _pTRAC_IK = boost::shared_ptr<TRAC_IK::TRAC_IK>(new 
            TRAC_IK::TRAC_IK( _rootlinkname,  _tiplinkname, "/robot_description", 0.005, 1e-5, TRAC_IK::Speed));
           //TRAC_IK::TRAC_IK( _rootlinkname,"motoman_robotiq_85_adapter_link")); //  _tiplinkname));
    }    catch (std::exception e) {
        std::cout << "MotomanSia20dTrak_IK::Init() Exception: " << e.what() << "\n";
    }    catch (const boost::system::system_error& ex) {
        std::cout << "MotomanSia20dTrak_IK::Init() Exception: " << ex.what() << "\n";
    }

}
#endif