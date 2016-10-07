

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
///////////////////////////////////////////////////////////////////////////////

RCS::Rotation FastKinematics::Convert2Rotation(double *eerot) {
    double q0 = (eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
    double q1 = (eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
    double q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
    double q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;

    if (q0 < 0.0f) {
        q0 = 0.0f;
    }

    if (q1 < 0.0f) {
        q1 = 0.0f;
    }

    if (q2 < 0.0f) {
        q2 = 0.0f;
    }

    if (q3 < 0.0f) {
        q3 = 0.0f;
    }
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);

    if ((q0 >= q1) && (q0 >= q2) && (q0 >= q3)) {
        q0 *= +1.0f;
        q1 *= SIGN(eerot[7] - eerot[5]);
        q2 *= SIGN(eerot[2] - eerot[6]);
        q3 *= SIGN(eerot[3] - eerot[1]);
    } else if ((q1 >= q0) && (q1 >= q2) && (q1 >= q3)) {
        q0 *= SIGN(eerot[7] - eerot[5]);
        q1 *= +1.0f;
        q2 *= SIGN(eerot[3] + eerot[1]);
        q3 *= SIGN(eerot[2] + eerot[6]);
    } else if ((q2 >= q0) && (q2 >= q1) && (q2 >= q3)) {
        q0 *= SIGN(eerot[2] - eerot[6]);
        q1 *= SIGN(eerot[3] + eerot[1]);
        q2 *= +1.0f;
        q3 *= SIGN(eerot[7] + eerot[5]);
    } else if ((q3 >= q0) && (q3 >= q1) && (q3 >= q2)) {
        q0 *= SIGN(eerot[3] - eerot[1]);
        q1 *= SIGN(eerot[6] + eerot[2]);
        q2 *= SIGN(eerot[7] + eerot[5]);
        q3 *= +1.0f;
    } else {
        throw std::runtime_error("Error while converting to quaternion! \n");
    }
    double r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;
    RCS::Rotation q(q0, q1, q2, q3);
    return q;
}
// Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
// Must use doubles, else lose precision compared to directly inputting the rotation matrix.
// Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp

void FastKinematics::Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot) {
    double qq1 = 2 * quat.x() * quat.x();
    double qq2 = 2 * quat.y() * quat.y();
    double qq3 = 2 * quat.z() * quat.z();
    eerot[3 * 0 + 0] = 1 - qq2 - qq3;
    eerot[3 * 0 + 1] = 2 * (quat.x() * quat.y() - quat.w() * quat.z());
    eerot[3 * 0 + 2] = 2 * (quat.x() * quat.z() + quat.w() * quat.y());
    eerot[3 * 1 + 0] = 2 * (quat.x() * quat.y() + quat.w() * quat.z());
    eerot[3 * 1 + 1] = 1 - qq1 - qq3;
    eerot[3 * 1 + 2] = 2 * (quat.y() * quat.z() - quat.w() * quat.x());
    eerot[3 * 2 + 0] = 2 * (quat.x() * quat.z() - quat.w() * quat.y());
    eerot[3 * 2 + 1] = 2 * (quat.y() * quat.z() + quat.w() * quat.x());
    eerot[3 * 2 + 2] = 1 - qq1 - qq2;
}

double FastKinematics::harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
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

void FastKinematics::getClosestSolution(const IkSolutionList<IkReal> &solutions,
        const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
    double mindist = DBL_MAX;
    int minindex = -1;
    std::vector<double> sol;
#if 0
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
#endif
}

RCS::Pose FastKinematics::FK(std::vector<double> joints) {
    // DO NOT Handle gearing of joints
    double eetrans[4];
    double eerot[9];

    // / Computes the end effector coordinates given the joint values using ikfast. This function is used to double check ik
    // Units? joint angles in radians or degree
    // Elsewhere: Returns the forward kinematic solution given the joint angles (in radians)
    ComputeFk(&joints[0], eetrans, eerot);

    RCS::Pose pose;
    pose.getOrigin().setX(eetrans[0]);
    pose.getOrigin().setY(eetrans[1]);
    pose.getOrigin().setZ(eetrans[2]);
    pose.setRotation(Convert2Rotation(eerot));
    return pose;
}
// http://docs.ros.org/jade/api/moveit_msgs/html/msg/PositionIKRequest.html
//http://docs.ros.org/hydro/api/ric_mc/html/GetPositionIK_8h_source.html

size_t FastKinematics::AllPoseToJoints(RCS::Pose & pose, std::vector<std::vector<double>> &joints) {

    // Inverse kinematics
    ikfast::IkSolutionList<double> solutions;

    // std::vector<double> vfree(GetNumFreeParameters());
    std::vector<double> vfree(1, 3);

    double eetrans[3] = {pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()};

    double eerot[9];
    Convert2RotationMatrix(pose.getRotation(), eerot);
#if 0
    LOG_DEBUG << << Globals.StrFormat("IKFAST IK");
    LOG_DEBUG << << Globals.StrFormat("Pos  X=%6.4f Y=%6.4f Z=%6.4f", eetrans[0], eetrans[1], eetrans[2]);
    LOG_DEBUG << << Globals.StrFormat("XROT I=%6.4f J=%6.4f K=%6.4f", eerot[0], eerot[1], eerot[2]);
    LOG_DEBUG << << Globals.StrFormat("ZROT I=%6.4f J=%6.4f K=%6.4f", eerot[6], eerot[7], eerot[8]);
#endif
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if (!bSuccess) {
        ROS_ERROR("Failed to get ik solution");
        return -1;
    }

    // There are no redundant joints, so no free dof

    //LOG_DEBUG <<  Globals.StrFormat("Found %d ik solutions:\n", (int) solutions.GetNumSolutions());
    std::vector<double> solvalues(GetNumJoints());

    for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const ikfast::IkSolutionBase<double> & sol = solutions.GetSolution(i);

#if 0
        LOG_DEBUG << Globals.StrFormat("sol%d (free=%d): ", (int) i, (int) sol.GetFree().size());
#endif
        std::vector<double> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

#if 0
        for (std::size_t j = 0; j < solvalues.size(); ++j) {
            //                std::cerr << Globals.StrFormat("%6.4f, ", Rad2Deg(solvalues[j]));
            LOG_DEBUG << Globals.StrFormat("%6.4f, ", solvalues[j]);
        }
        LOG_DEBUG << Globals.StrFormat("\n");
#endif

        // No gearing done
        std::vector<double> jnts;
        for (std::size_t j = 0; j < solvalues.size(); ++j) {
            jnts.push_back(solvalues[j]);
        }
        joints.push_back(jnts);
    }
    return solutions.GetNumSolutions();
}

Eigen::VectorXd FastKinematics::ConvertJoints(std::vector<double> v) {
    Eigen::VectorXd p(v.size());
    for (size_t i = 0; i < v.size(); i++)
        p(i) = v[i];
    return p;
}

std::vector<double> FastKinematics::ConvertJoints(Eigen::VectorXd ev) {
    std::vector<double> v;
    for (int i = 0; i < ev.size(); i++)
        v.push_back(ev(i));
    return v;
}

std::vector<double> FastKinematics::NearestJoints(
        std::vector<double> oldjoints,
        std::vector<std::vector<double>> &newjoints) {
    std::vector<double> finaljoints;
    Eigen::VectorXd oldjointvec = ConvertJoints(oldjoints);
    double min = std::numeric_limits<double>::infinity();
    size_t index = 0;
    for (size_t i = 0; i < newjoints.size(); i++) {
        Eigen::VectorXd newjointvec = ConvertJoints(newjoints[i]);
        double diff = (oldjointvec - newjointvec).norm();
        if (diff < min) {
            min = diff;
            index = i;
        }
    }
    // save "best" solution - closset ignoring importance of wrist
    finaljoints.insert(finaljoints.begin(), newjoints[index].begin(), newjoints[index].end());
    return finaljoints;
}

std::vector<double> FastKinematics::IK(RCS::Pose pose,
        std::vector<double> oldjoints) {
    std::vector<std::vector<double>> allsolutions;
    size_t bFlag = AllPoseToJoints(pose, allsolutions);

    return NearestJoints(oldjoints, allsolutions);
    //response.error_code.val == response.error_code.SUCCES
    //return response.solution.joint_state.position;
}

std::vector<double> FastKinematics::IK(RCS::Pose pose,
        std::vector<double> minrange, std::vector<double> maxrange) {
    std::vector<std::vector<double>> allsolutions;
    size_t bFlag = AllPoseToJoints(pose, allsolutions);
    for (size_t i = 0; i < allsolutions.size(); i++) {
        bool bFlag = true;
        for (size_t j = 0; j < allsolutions[0].size(); j++) {
            if (allsolutions[i][j] < minrange[j] || allsolutions[i][j] > maxrange[j])
                bFlag = false;
        }
        if (bFlag)
            return allsolutions[i];

    }
    // just pick one
    size_t n = rand() % allsolutions.size();
    return allsolutions[n];
}

bool FastKinematics::IsSingular(RCS::Pose pose, double threshold) {
    return false;
}

void FastKinematics::Init(ros::NodeHandle &nh) {
    armkin = boost::shared_ptr<::Kinematics>(new ::Kinematics());
    armkin->init(nh, _tiplinkname, _rootlinkname);
    moveit_msgs::GetKinematicSolverInfo::Request request;
    moveit_msgs::GetKinematicSolverInfo::Response response;
    armkin->getFKSolverInfo(request, response);
    joint_names.clear();
    link_names.clear();
    num_joints = response.kinematic_solver_info.joint_names.size();
    for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
        joint_names.push_back(response.kinematic_solver_info.joint_names[i]);
    }
    for (unsigned int i = 0; i < response.kinematic_solver_info.link_names.size(); i++) {
        link_names.push_back(response.kinematic_solver_info.link_names[i]);
    }
    for (int i = 0; i < armkin->joint_min.rows(); i++)
        joint_min.push_back(armkin->joint_min(i));
    for (int i = 0; i < armkin->joint_max.rows(); i++)
        joint_max .push_back(armkin->joint_max(i));
}

void FastKinematics::VerifyLimits(std::vector<double> joints) {
    for (size_t i = 0; i < joints.size(); i++)
        if (joints[i] < joint_min[i] || joints[i] > joint_max[i])
            ROS_ERROR_STREAM("Verify Joint Limits Joint" << joint_names[i] << "out of range");

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

void MotomanSia20d::getClosestSolution(const IkSolutionList<IkReal> &solutions, const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
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