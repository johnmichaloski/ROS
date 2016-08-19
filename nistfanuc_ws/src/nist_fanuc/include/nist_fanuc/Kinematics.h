

#pragma once

#include "RCS.h"
#include "Globals.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

/**
 * \brief The IKinematics provides is an abstract class with pure virtual functions that are
 * overriden by actual kinematic implementations.
 * */
class IKinematics {
protected:
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
public:
    size_t NumJoints() { 
        assert(joint_names.size()!=0);
        return joint_names.size(); 
    }
    /*!
     * \brief GetJointValues returns latest reading of end effector.
     * \return vector of joint values in doubles.
     */
    virtual std::vector<double> GetJointValues() = 0;
    /*!
     * \brief SetJointValues sets the latest joint values of the robot.
     * \param vector of all robot joint values in doubles.
     */
    virtual void SetJointValues(std::vector<double> joint_values) = 0;
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
    virtual void Init(ros::NodeHandle &nh) {
    }

    virtual std::vector<std::string> JointNames() {
        return std::vector<std::string>();
    }

    virtual std::vector<std::string> LinkNames() {
        return std::vector<std::string>();
    }
    // Fixme: add joint limits

    virtual JointState ZeroJointState() {
        return JointState();
    }

    virtual JointState UpdateJointState(std::vector<uint64_t> jointnums, JointState oldjoints, JointState njoints) {
        return JointState();
    }

};
typedef boost::shared_ptr<IKinematics> IKinematicsSharedPtr;

/**
 * \brief The DummyKinematics class  instantiates the IKinematics abstract class
 * and fills in the pure virtual functions with dummy methods.
 * */
class DummyKinematics : public IKinematics {
public:

    virtual std::vector<double> GetJointValues() {
        std::vector<double> joints = ToVector<double>(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        return joints;
    }

    virtual void SetJointValues(std::vector<double> joint_values) {
    }

    virtual RCS::Pose FK(std::vector<double> jv) {
        return RCS::Pose();
    }

    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) {
        std::vector<std::vector<double> > newjoints;
        size_t solutions = AllPoseToJoints(pose, newjoints);
        assert(solutions > 0);
        return NearestJoints(oldjoints, newjoints);

    }

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        newjoints.push_back(ToVector<double>(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        return 1;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        assert(newjoints.size() > 0);
        return newjoints[0];
    }

    virtual bool IsSingular(RCS::Pose & pose, double threshold) {
        return true;
    }

};
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/rate.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Header.h>


typedef moveit::planning_interface::MoveItErrorCode RosErrorCode;

/**
 * \brief The RosKinematics class  instantiates the IKinematics abstract class
 * and fills in the pure virtual functions with Descartes kinematic methods.
 * */
class RosKinematics : public IKinematics {
public:
    RosKinematics();
    virtual void Init(
            std::string groupname,
            std::string eelinkname);
    virtual std::vector<double> GetJointValues();
    void SetJointValues(std::vector<double> joint_values);
    virtual RCS::Pose FK(std::vector<double> jv);
    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints);
    bool SatisfiesBounds();
    void EnforceBounds();

    virtual bool IsSingular(RCS::Pose & pose, double threshold) {
        return true;
    }

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        newjoints.push_back(ToVector<double>(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        return 1;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        assert(newjoints.size() > 0);
        return newjoints[0];
    }

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<double> joint_values;
    std::vector<std::string> joint_names;
    std::string _groupname;
    std::string _eelinkname;
    //ros::NodeHandle &node;
    bool _bInit;
    boost::mutex kinmutex;
};

/**
 * \brief The MoveitKinematics class  instantiates the IKinematics abstract class
 * and fills in the pure virtual functions with Moveit kinematic methods.
 * */
class MoveitKinematics : public IKinematics {
public:
    MoveitKinematics(ros::NodeHandle &nh);
    virtual std::vector<double> GetJointValues();
    virtual void SetJointValues(std::vector<double> joint_values);
    virtual RCS::Pose FK(std::vector<double> jv);
    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints);

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        return 0;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        return std::vector<double>();
    }
    virtual void Init(
            std::string groupname,
            std::string eelinkname);

    virtual bool IsSingular(RCS::Pose & pose,
            double threshold);
    /////////////////////////////////////////
    boost::shared_ptr<moveit::planning_interface::MoveGroup> group;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<double> joint_values;
    std::vector<std::string> joint_names;
    std::string _groupname;
    std::string _eelinkname;
    ros::NodeHandle &_nh;
    bool _bInit;
    boost::mutex kinmutex;
};

//https://github.com/davetcoleman/kdlc_kinematic_plugin/blob/master/src/kdlc_kinematics_plugin.cpp
// http://wiki.ros.org/arm_navigation/Tutorials/Running%20arm%20navigation%20on%20non-PR2%20arm
// https://github.com/ros-planning/moveit_kinematics_tests/blob/kinetic-devel/kinematics_base_test/src/test_kinematics_plugin.cpp
// https://github.com/IDSCETHZurich/re_trajectory-generator/blob/master/poseToOrocos/src/poseStampedLoop.cpp
#include <boost/shared_ptr.hpp>
#include "arm_kinematics.h"
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>

class ArmKinematics : public IKinematics {
public:
    boost::shared_ptr<::Kinematics> armkin;
    std::vector< double> jointvalues;
    size_t num_joints;
    std::string _groupname;
    std::string _eelinkname;

    virtual std::vector<double> GetJointValues() {
        return jointvalues;
    }

    virtual void SetJointValues(std::vector<double> joint_values) {
        jointvalues = joint_values;
    }

    virtual RCS::Pose FK(std::vector<double> jv) {
        moveit_msgs::GetPositionFK::Response response;
        moveit_msgs::GetPositionFK::Request request;
        request.fk_link_names = link_names;
        request.robot_state.joint_state.header.frame_id = "base_link";
        request.robot_state.joint_state.name = joint_names;
        request.robot_state.joint_state.position = jv;
        request.header.frame_id = "base_link";
        armkin->getPositionFK(request, response);
        RCS::Pose  pose;
        RCS::ConvertGeometryPose2TfPose(response.pose_stamped[0].pose, pose);
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
        RCS::ConvertTfPose2GeometryPose(pose, request.ik_request.pose_stamped.pose);

        //request.pose_stamped.pose = pose;
        request.ik_request.attempts = 1;
        armkin->getPositionIK(request, response);
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

    virtual std::vector<std::string> JointNames() {
        return joint_names;
    }

    virtual std::vector<std::string> LinkNames() {
        return link_names;
    }
    // Fixme: add joint limits

    virtual JointState ZeroJointState() {
        JointState joints;
        for (size_t i = 0; i < joint_names.size(); i++) {
            joints.position.push_back(0.0);
            joints.velocity.push_back(DEFAULT_JOINT_MAX_VEL);
            joints.effort.push_back(DEFAULT_JOINT_MAX_ACCEL);
        }
        return joints;
    }

    virtual JointState UpdateJointState(std::vector<uint64_t> jointnums,
            JointState oldjoints,
            JointState njoints) {
        JointState joints = oldjoints;
        // Check each joint, to see if joint is being actuated, if so, change goal position
        for (size_t i = 0; i < jointnums.size(); i++) {
            size_t n = jointnums[i];
            joints.position[n] = njoints.position[n]; // joint numbers already adjusted from CRCL to rcs model
            joints.velocity[n] = njoints.velocity[n];
            joints.effort[n] = njoints.effort[n];
        }
        return joints;
    }
};