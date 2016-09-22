

#pragma once

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include "arm_kinematics.h"

#include "RCS.h"
#include "Globals.h"
#include "Conversions.h"
#include "Debug.h"


#define _USE_MATH_DEFINES
#include <math.h>

/***
 * \brief ArmConfiguration provides a class to handle manipulator arm configuration.
 * using a bit mask determines a configuration. Given a configuration, a min and max vector 
 * of angles can be used to determine the desired solution, when mutliple solutions are available (e.g. ikfast).
 */
class IArmConfiguration {
protected:
    int _config;
    size_t _size;
    std::vector<double> _min;
    std::vector<double> _max;
    bool bConfig;
public:
    static const int BASE_FLIP = 0x01;
    static const int ARM_BACKWARD = 0x02; // ex. base flip, arm backwards
    static const int SHOULDER_RIGHT = 0x040;
    static const int SHOULDER_DOWN = 0x04;
    static const int SHOULDER_UP = 0x040;

    static const int ELBOW_DOWN = 0x08;
    static const int ELBOW_UP = 0x080;
    static const int FOREARM_UP = 0x10;
    static const int FOREARM_DOWN = 0x100;

    static const int WRIST_NORMAL = 0x200;
    static const int WRIST_FLIP = 0x400;

    static const int SINGULAR = -1;

    IArmConfiguration() : _config(0), _size(0), bConfig(false) {
    }

    /***
     * \brief Configure provides an overloaded virtual function to assign joint ranges based on configuration.
     * \param config arm configuration mask
     * \param size number of joints in arm
     * \param min reference to vector for storing min values to match configuration
     * \param max reference to vector for storing max values to match configuration
     */
    virtual void Configure(int config, size_t size,
            std::vector<double>& min,
            std::vector<double> &max) {

    }

    void Range(std::vector<double> & min, std::vector<double> &max) {
        if (!bConfig)
            throw std::runtime_error("ArmConfiguration not configured\n");
        min = _min;
        max = _max;
    }
};

class FanucLRmate200iD : public IArmConfiguration {
public:

    FanucLRmate200iD() {
        Configure(SHOULDER_DOWN | FOREARM_UP | ELBOW_DOWN | WRIST_NORMAL, 6, _min, _max);

    }
    virtual void Configure(int config, size_t size,
            std::vector<double>& min,
            std::vector<double> &max);

};



/**
 * \brief The IKinematics provides is an abstract class with pure virtual functions that are
 * overriden by actual kinematic implementations.
 * */
class IKinematics {
protected:
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::vector< double> jointvalues;
    std::vector< double> joint_min;
    std::vector< double> joint_max;
    std::vector< double> hint;
    size_t num_joints;
    std::string _groupname;
    std::string _eelinkname;
public:
    boost::shared_ptr<::Kinematics> armkin;

    size_t NumJoints() {
        assert(joint_names.size() != 0);
        return joint_names.size();
    }

    virtual void SetHint(std::vector< double> hint) {
        this->hint = hint;
    }
    //    RCS:Pose GetJointTransform(std::string jointname){
    //    boost::shared_ptr<const urdf::Joint> urdf_joint = armkin->robot_model->getJoint(jointname);
    //    }

    virtual std::vector<std::string> JointNames() {
        return joint_names;
    }

    virtual std::vector<std::string> LinkNames() {
        return link_names;
    }

    void VerifyLimits(std::vector<double> joints) {
    }

    /*!
     * \brief GetJointValues returns latest reading of end effector.
     * \return vector of joint values in doubles.
     */
    virtual std::vector<double> GetJointValues() {
        return jointvalues;
    }

    /*!
     * \brief SetJointValues sets the latest joint values of the robot.
     * \param vector of all robot joint values in doubles.
     */
    virtual void SetJointValues(std::vector<double> joint_values) {
        jointvalues = joint_values;
    }
    /*!
     * \brief FK performs the forward kinematics using the joint values of the robot provided.
     * \param vector of all robot joint values in doubles.
     * \return corresponding Cartesian robot pose of end  effector.
     */
    virtual RCS::Pose FK(std::vector<double> jv) = 0;
    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  optional seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) = 0;

    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  optional seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> minrange, std::vector<double> maxrange) {
        return std::vector<double>();
    }

    /*!
     * \brief AllPoseToJoints solves  the inverse kinematics to find all solutions using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  vector of double vectos to hold all the IK joint solutions.
     * \return number of solutions found.
     */
    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) = 0;
    /*!
     * \brief NearestJoints finds the joint set that is closest to the old joints.
     * \param  old seed  joint values to use as best guess for IK joint values.
     * \param  vector of double vectos that holds all the IK joint solutions.
     * \return vector of doubles with closest set to seed joints.
     */
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) = 0;

    /*!
     * \brief Init is necessary for ROS to initialize it kinematics using robot model .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */

    virtual void Init(
            std::string groupname,
            std::string eelinkname) {
        _groupname = groupname;
        _eelinkname = eelinkname;
    }

    /*!
     * \brief Returns true if the determinant of the jacobian is near zero. .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */
    virtual bool IsSingular(RCS::Pose & pose, double threshold) {
        return false;
    }

    /*!
     * \brief Initialize kinematics using robot_description to fill parameters .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */
    virtual void Init(ros::NodeHandle & nh) {
    }

    virtual JointState UpdateJointState(std::vector<uint64_t> jointnums,
            JointState oldjoints,
            JointState njoints) {
        JointState joints = oldjoints;
        if (joints.velocity.size() != joints.position.size())
            joints.velocity.resize(joints.position.size(), 0.0);
        if (joints.effort.size() != joints.position.size())
            joints.effort.resize(joints.position.size(), 0.0);
       
        // Check each joint, to see if joint is being actuated, if so, change goal position
        for (size_t i = 0; i < jointnums.size(); i++) {
            size_t n = jointnums[i];
            joints.position[n] = njoints.position[n]; // joint numbers already adjusted from CRCL to rcs model
            joints.velocity[n] = njoints.velocity[n];
            joints.effort[n] = njoints.effort[n];
            joints.effort[n] = 0.0;
       }
        return joints;
    }

    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            std::vector<double> &min,
            std::vector<double> &max) {
        return std::vector<double>();
    }

    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            boost::shared_ptr<IArmConfiguration> config) {
        return std::vector<double>();
    }
};
typedef boost::shared_ptr<IKinematics> IKinematicsSharedPtr;


//https://github.com/davetcoleman/kdlc_kinematic_plugin/blob/master/src/kdlc_kinematics_plugin.cpp
// http://wiki.ros.org/arm_navigation/Tutorials/Running%20arm%20navigation%20on%20non-PR2%20arm
// https://github.com/ros-planning/moveit_kinematics_tests/blob/kinetic-devel/kinematics_base_test/src/test_kinematics_plugin.cpp
// https://github.com/IDSCETHZurich/re_trajectory-generator/blob/master/poseToOrocos/src/poseStampedLoop.cpp
#include <boost/shared_ptr.hpp>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>

class ArmKinematics : public IKinematics {
public:

    virtual RCS::Pose FK(std::vector<double> jv) {
        moveit_msgs::GetPositionFK::Response response;
        moveit_msgs::GetPositionFK::Request request;
        request.fk_link_names = link_names;
        request.robot_state.joint_state.header.frame_id = "base_link";
        request.robot_state.joint_state.name = joint_names;
        request.robot_state.joint_state.position = jv;
        request.header.frame_id = "base_link";
        armkin->getPositionFK(request, response);
        RCS::Pose pose;
        Conversion::GeometryPose2TfPose(response.pose_stamped[0].pose, pose);
        return pose;
    }
    // http://docs.ros.org/jade/api/moveit_msgs/html/msg/PositionIKRequest.html
    //http://docs.ros.org/hydro/api/ric_mc/html/GetPositionIK_8h_source.html

    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) {
        moveit_msgs::GetPositionIK::Response response;
        moveit_msgs::GetPositionIK::Request request;
        request.ik_request.pose_stamped.header.stamp = ros::Time::now();
        request.ik_request.ik_link_name = _eelinkname;
        request.ik_request.group_name = "manipulator";
        request.ik_request.robot_state.joint_state.name = joint_names;
        request.ik_request.ik_link_names = link_names;
        request.ik_request.robot_state.joint_state.position = oldjoints;
        Conversion::TfPose2GeometryPose(pose, request.ik_request.pose_stamped.pose);

        //request.pose_stamped.pose = pose;
        request.ik_request.attempts = 1;
        armkin->getPositionIK(request, response);
        //response.error_code.val == response.error_code.SUCCES
        return response.solution.joint_state.position;
    }

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        return 0;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        ROS_ERROR("ArmKinematics::NearestJoints() not implemented");
        return std::vector<double>();
    }

    virtual void Init(
            std::string groupname,
            std::string eelinkname) {
        _groupname = groupname;
        _eelinkname = eelinkname;
    }

    virtual bool IsSingular(RCS::Pose & pose, double threshold) {
        return false;
    }

    virtual void Init(ros::NodeHandle &nh) {
        armkin = boost::shared_ptr<::Kinematics>(new ::Kinematics());
        armkin->init(nh);
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
    }
};

#include "ikfast.h"
#include <algorithm>

class FastKinematics : public IKinematics {

    static double SIGN(double x) {
        return ( x >= 0.0f) ? +1.0f : -1.0f;
    }

    static double NORM(double a, double b, double c, double d) {
        return sqrt(a * a + b * b + c * c + d * d);
    }
    // Convert rotation matrix to quaternion (Daisuke Miyazaki)
    // http://pastebin.com/NikwbL3k

    static RCS::Rotation Convert2Rotation(IkReal *eerot) {
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

    static void Convert2RotationMatrix(const RCS::Rotation & quat, IkReal *eerot) {
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
public:

    virtual RCS::Pose FK(std::vector<double> jv) {
        // Handle gearing of joints
        std::vector<double> joints;
        joints.insert(joints.begin(), jv.begin(), jv.end());
        //joints(1) = thetas[1] - M_PI_2;
        joints[2] += jv[1];

        // IkReal j[6]={ 0.0, 0.0, 0.0, 0.0, 0.0};
        IkReal eetrans[4];
        IkReal eerot[9];

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

    virtual size_t AllPoseToJoints(RCS::Pose & pose, std::vector<std::vector<double>> &joints) {
        // Inverse kinematics
        ikfast::IkSolutionList<IkReal> solutions;

        std::vector<IkReal> vfree(GetNumFreeParameters());
        IkReal eetrans[3] = {pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()}; // + .33};

        IkReal eerot[9];
        Convert2RotationMatrix(pose.getRotation(), eerot);
#ifdef DEBUG
        std::cout << Globals.StrFormat("IKFAST IK\n");
        std::cout << Globals.StrFormat("Pos  X=%6.4f Y=%6.4f Z=%6.4f\n", eetrans[0], eetrans[1], eetrans[2]);
        std::cout << Globals.StrFormat("XROT I=%6.4f J=%6.4f K=%6.4f\n", eerot[0], eerot[1], eerot[2]);
        std::cout << Globals.StrFormat("ZROT I=%6.4f J=%6.4f K=%6.4f\n", eerot[6], eerot[7], eerot[8]);
#endif
        bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

        if (!bSuccess) {
            std::cerr << Globals.StrFormat("Failed to get ik solution\n");
            return -1;
        }

        // There are no redundant joints, so no free dof

        std::cerr << Globals.StrFormat("Found %d ik solutions:\n", (int) solutions.GetNumSolutions());
        std::vector<IkReal> solvalues(GetNumJoints());

        for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
            const ikfast::IkSolutionBase<IkReal> & sol = solutions.GetSolution(i);

#ifdef DEBUG
            std::cerr << Globals.StrFormat("sol%d (free=%d): ", (int) i, (int) sol.GetFree().size());
#endif
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

#ifdef DEBUG
            for (std::size_t j = 0; j < solvalues.size(); ++j) {
                std::cerr << Globals.StrFormat("%6.4f, ", Rad2Deg(solvalues[j]));
            }
            std::cerr << Globals.StrFormat("\n");
#endif

            std::vector<double> jnts;

            for (std::size_t j = 0; j < solvalues.size(); ++j) {
                jnts.push_back(solvalues[j]);
            }
            jnts[2] -= jnts[1];
            joints.push_back(jnts);
        }
        return solutions.GetNumSolutions();
    }

    Eigen::VectorXd ConvertJoints(std::vector<double> v) {
        Eigen::VectorXd p(v.size());
        for (size_t i = 0; i < v.size(); i++)
            p(i) = v[i];
        return p;
    }

    std::vector<double> ConvertJoints(Eigen::VectorXd ev) {
        std::vector<double> v;
        for (int i = 0; i < ev.size(); i++)
            v.push_back(ev(i));
        return v;
    }

    virtual std::vector<double> NearestJoints(
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

    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) {
        std::vector<std::vector<double>> allsolutions;
        size_t bFlag = AllPoseToJoints(pose, allsolutions);

        return NearestJoints(oldjoints, allsolutions);
        //response.error_code.val == response.error_code.SUCCES
        //return response.solution.joint_state.position;
    }

    virtual std::vector<double> IK(RCS::Pose & pose,
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

    virtual void Init(
            std::string groupname,
            std::string eelinkname) {
        _groupname = groupname;
        _eelinkname = eelinkname;
    }

    virtual bool IsSingular(RCS::Pose & pose, double threshold) {
        return false;
    }

    virtual void Init(ros::NodeHandle &nh) {
        armkin = boost::shared_ptr<::Kinematics>(new ::Kinematics());
        armkin->init(nh);
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

    void VerifyLimits(std::vector<double> joints) {
        for (size_t i = 0; i < joints.size(); i++)
            if (joints[i] < joint_min[i] || joints[i] > joint_max[i])
                ROS_ERROR_STREAM("Verify Joint Limits Joint" << joint_names[i] << "out of range");

    }
    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            std::vector<double> &min,
            std::vector<double> &max);

};
#include "fanuc_lrmate200id.h"

class FanucLrMate200idKinematics : public IKinematics {
    fanuc_lrmate200id fanuckin;
    tf::Pose base;
public:

    FanucLrMate200idKinematics() {
        base = tf::Pose(tf::Quaternion(0, 0, 0, 1),
                tf::Vector3(0.0, 0.0, 0.330));
    }

    virtual RCS::Pose FK(std::vector<double> jv) {

        tf::Pose pose = fanuckin.fanuc_lrmate200id_kin_fwd(&jv[0]);
        return base*pose;
        //return pose;
    }

    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) {
        tf::Pose base(tf::Quaternion(0, 0, 0, 1),
                tf::Vector3(0.0, 0.0, 0.330));
        pose = base.inverse() * pose;
        std::vector<double> joints = fanuckin.fanuc_lrmate200id_kin_inv(pose);
        return joints;
    }

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        return 0;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        ROS_ERROR("FanucLrMate200idKinematics::NearestJoints() not implemented");
        return std::vector<double>();
    }

    virtual void Init(ros::NodeHandle &nh) {
        armkin = boost::shared_ptr<::Kinematics>(new ::Kinematics());
        armkin->init(nh);
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
    }



};