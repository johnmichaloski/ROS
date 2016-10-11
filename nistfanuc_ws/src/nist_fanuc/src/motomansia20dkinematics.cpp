

// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.


#include "Kinematics.h"
#include <eigen_conversions/eigen_msg.h>
#include "RosConversions.h"
#include <iostream>
#include "Conversions.h"
#include "Globals.h"
#include "Debug.h"
#include <boost/format.hpp>
#include "BLogging.h"

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_NAMESPACE MotomanSia20d

#include "Motoman/ikfast.h"
#include <algorithm>
using namespace  ikfast;
using namespace MotomanSia20d;


///////////////////////////////////////////////////////////////////////////////
using namespace MotomanSia20d;

static     void getClosestSolution(const IkSolutionList<double> &solutions, 
    const std::vector<double> &ik_seed_state, 
    std::vector<double> &solution)  ;
    

RCS::Rotation MotomanSia20dFastKinematics::Convert2Rotation(double *eerot) {
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

void MotomanSia20dFastKinematics::Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot) {
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

double MotomanSia20dFastKinematics::harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
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

void getClosestSolution(const IkSolutionList<double> &solutions,
        const std::vector<double> &ik_seed_state, std::vector<double> &solution)  {
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

RCS::Pose MotomanSia20dFastKinematics::FK(std::vector<double> joints) {
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

size_t MotomanSia20dFastKinematics::AllPoseToJoints(RCS::Pose & pose, std::vector<std::vector<double>> &joints) {

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

    TODO(Fix Singularity issue in MotomanSia20dFastKinematics::AllPoseToJoints)
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

Eigen::VectorXd MotomanSia20dFastKinematics::ConvertJoints(std::vector<double> v) {
    Eigen::VectorXd p(v.size());
    for (size_t i = 0; i < v.size(); i++)
        p(i) = v[i];
    return p;
}

std::vector<double> MotomanSia20dFastKinematics::ConvertJoints(Eigen::VectorXd ev) {
    std::vector<double> v;
    for (int i = 0; i < ev.size(); i++)
        v.push_back(ev(i));
    return v;
}

std::vector<double> MotomanSia20dFastKinematics::NearestJoints(
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

std::vector<double> MotomanSia20dFastKinematics::IK(RCS::Pose pose,
        std::vector<double> oldjoints) {
    std::vector<std::vector<double>> allsolutions;
    size_t bFlag = AllPoseToJoints(pose, allsolutions);

    return NearestJoints(oldjoints, allsolutions);
    //response.error_code.val == response.error_code.SUCCES
    //return response.solution.joint_state.position;
}

std::vector<double> MotomanSia20dFastKinematics::IK(RCS::Pose pose,
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

bool MotomanSia20dFastKinematics::IsSingular(RCS::Pose pose, double threshold) {
    return false;
}

void MotomanSia20dFastKinematics::Init(ros::NodeHandle &nh) {
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

void MotomanSia20dFastKinematics::VerifyLimits(std::vector<double> joints) {
    for (size_t i = 0; i < joints.size(); i++)
        if (joints[i] < joint_min[i] || joints[i] > joint_max[i])
            ROS_ERROR_STREAM("Verify Joint Limits Joint" << joint_names[i] << "out of range");

}
std::vector<double> MotomanSia20dFastKinematics::FindBoundedSolution(std::vector<std::vector<double>> &solutions,
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

#include "Motoman/ikfast_sia20d_manipulator.cpp"

