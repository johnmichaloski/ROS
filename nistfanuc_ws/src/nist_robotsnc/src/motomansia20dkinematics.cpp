

// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.


#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <algorithm>
#include <boost/format.hpp>

#include "nist_robotsnc/Kinematics.h"
#include "nist_robotsnc/Conversions.h"
#include "nist_robotsnc/Globals.h"
#include "nist_robotsnc/Debug.h"
#include "nist_robotsnc/NIST/Boost.h"
#include "nist_robotsnc/MotionException.h"
#include "nist_robotsnc/Controller.h"

using namespace RCS;


#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_NAMESPACE MotomanSia20d

#include "nist_robotsnc/Motoman/ikfast.h"
using namespace  ikfast;
using namespace MotomanSia20d;


///////////////////////////////////////////////////////////////////////////////
using namespace MotomanSia20d;

static     void getClosestSolution(const IkSolutionList<double> &solutions, 
    const std::vector<double> &ik_seed_state, 
    std::vector<double> &solution)  ;
 
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
    tf::Matrix3x3 m(eerot[3 * 0 + 0], eerot[3 * 0 + 1], eerot[3 * 0 + 2],
            eerot[3 * 1 + 0], eerot[3 * 1 + 1], eerot[3 * 1 + 2],
            eerot[3 * 2 + 0], eerot[3 * 2 + 1], eerot[3 * 2 + 2]);
    pose.setBasis(m);
    return pose;
}
// http://docs.ros.org/jade/api/moveit_msgs/html/msg/PositionIKRequest.html
//http://docs.ros.org/hydro/api/ric_mc/html/GetPositionIK_8h_source.html

size_t MotomanSia20dFastKinematics::AllIK(RCS::Pose & pose, std::vector<std::vector<double>> &joints) {

    // Inverse kinematics
    ikfast::IkSolutionList<double> solutions;

    // std::vector<double> vfree(GetNumFreeParameters());
    std::vector<double> vfree(1, 3);

    double eetrans[3] = {pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()};

    double eerot[9];
    tf::Matrix3x3 m3x3 = pose.getBasis();
    eerot[3 * 0 + 0] = m3x3[0][0];
    eerot[3 * 0 + 1] = m3x3[0][1];
    eerot[3 * 0 + 2] = m3x3[0][2];
    eerot[3 * 1 + 0] = m3x3[1][0];
    eerot[3 * 1 + 1] = m3x3[1][1];
    eerot[3 * 1 + 2] = m3x3[1][2];
    eerot[3 * 2 + 0] = m3x3[2][0];
    eerot[3 * 2 + 1] = m3x3[2][1];
    eerot[3 * 2 + 2] = m3x3[2][2];  
#if 0
    LOG_DEBUG << << Globals.StrFormat("IKFAST IK");
    LOG_DEBUG << << Globals.StrFormat("Pos  X=%6.4f Y=%6.4f Z=%6.4f", eetrans[0], eetrans[1], eetrans[2]);
    LOG_DEBUG << << Globals.StrFormat("XROT I=%6.4f J=%6.4f K=%6.4f", eerot[0], eerot[1], eerot[2]);
    LOG_DEBUG << << Globals.StrFormat("ZROT I=%6.4f J=%6.4f K=%6.4f", eerot[6], eerot[7], eerot[8]);
#endif
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    //TODO(Fix Singularity issue in MotomanSia20dFastKinematics::AllPoseToJoints)
    if (!bSuccess) {
        LOG_DEBUG <<"Failed to get ik solution:"<<RCS::DumpPoseSimple(pose);
        throw MotionException(10, _nc->Name().c_str());
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
    GetClosestSolution(newjoints,oldjoints, finaljoints);
    return finaljoints;
#if 0
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
#endif
}

std::vector<double> MotomanSia20dFastKinematics::IK(RCS::Pose pose,
        std::vector<double> oldjoints) {
    std::vector<std::vector<double>> allsolutions;
    size_t bFlag = AllIK(pose, allsolutions);

    return NearestJoints(oldjoints, allsolutions);
    //response.error_code.val == response.error_code.SUCCES
    //return response.solution.joint_state.position;
}

bool MotomanSia20dFastKinematics::IsSingular(RCS::Pose pose, double threshold) {
    return false;
}

void MotomanSia20dFastKinematics::VerifyLimits(std::vector<double> joints) {
    for (size_t i = 0; i < joints.size(); i++)
        if (joints[i] < joint_min[i] || joints[i] > joint_max[i])
            ROS_ERROR_STREAM("Verify Joint Limits Joint" << joint_names[i] << "out of range");

}

#include "Motoman/ikfast_sia20d_manipulator.cpp"

