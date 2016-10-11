

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
 * 
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
    std::string _tiplinkname;
    std::string _rootlinkname;
    std::string prefix;
    tf::Pose baseoffset;
public:
    boost::shared_ptr<::Kinematics> armkin;

    std::string & Prefix() {
        return prefix;
    }

    tf::Pose  baseOffset() {
        return baseoffset;
    }
    tf::Pose  invBaseOffset() {
        return baseoffset.inverse();
    }
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
    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> oldjoints) = 0;

    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  optional seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual std::vector<double> IK(RCS::Pose  pose,
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
            std::string tiplinkname,
            std::string rootlinkname) {
        _groupname = groupname;
        _tiplinkname = tiplinkname;
        _rootlinkname = rootlinkname;
    }

    /*!
     * \brief Returns true if the determinant of the jacobian is near zero. .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */
    virtual bool IsSingular(RCS::Pose  pose, double threshold) {
        return false;
    }

    /*!
     * \brief Initialize kinematics using robot_description to fill parameters .
     * \param  nh ros node handle of node.
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
            size_t n = jointnums[i]; // should already have indexes -1;
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
// http://docs.ros.org/jade/api/moveit_msgs/html/msg/PositionIKRequest.html
//http://docs.ros.org/hydro/api/ric_mc/html/GetPositionIK_8h_source.html
#include <boost/shared_ptr.hpp>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>

class ArmKinematics : public IKinematics {
protected:

public:

    ArmKinematics(std::string prefix, tf::Pose baseoffset) {
        this->prefix = prefix;
        this->baseoffset=baseoffset;
      
    }
    virtual RCS::Pose FK(std::vector<double> jv) {
        moveit_msgs::GetPositionFK::Response response;
        moveit_msgs::GetPositionFK::Request request;
        request.fk_link_names = link_names;
        request.robot_state.joint_state.header.frame_id = _rootlinkname; // prefix + "base_link";
        request.robot_state.joint_state.name = joint_names;
        request.robot_state.joint_state.position = jv;
        request.header.frame_id =  _rootlinkname; //prefix + "base_link";
        armkin->getPositionFK(request, response);
        RCS::Pose pose;
        Conversion::GeometryPose2TfPose(response.pose_stamped[0].pose, pose);
        return pose;
    }

    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> oldjoints) {
        moveit_msgs::GetPositionIK::Response response;
        moveit_msgs::GetPositionIK::Request request;
        request.ik_request.pose_stamped.header.stamp = ros::Time::now();
        request.ik_request.ik_link_name = _tiplinkname;
        request.ik_request.group_name = "manipulator";
        request.ik_request.robot_state.joint_state.name = joint_names;
        request.ik_request.ik_link_names = link_names;
        request.ik_request.robot_state.joint_state.position = oldjoints;
        request.ik_request.timeout = ros::Duration(10.0);
        request.ik_request.pose_stamped.header.frame_id = _rootlinkname;
        
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


    virtual bool IsSingular(RCS::Pose  pose, double threshold) {
        return false;
    }

    virtual void Init(ros::NodeHandle &nh) {
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
    }
};


class MotomanSia20dFastKinematics : public IKinematics {
    static double SIGN(double x) {
        return ( x >= 0.0f) ? +1.0f : -1.0f;
    }

    static double NORM(double a, double b, double c, double d) {
        return sqrt(a * a + b * b + c * c + d * d);
    }
    // Convert rotation matrix to quaternion (Daisuke Miyazaki)
    // http://pastebin.com/NikwbL3k

    static RCS::Rotation Convert2Rotation(double *eerot) ;
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    // Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp

    static void Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot);

    double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;


    
public:

    virtual RCS::Pose FK(std::vector<double> joints);
 
    virtual size_t AllPoseToJoints(RCS::Pose & pose, 
    std::vector<std::vector<double>> &joints) ;
    Eigen::VectorXd ConvertJoints(std::vector<double> v) ;
    std::vector<double> ConvertJoints(Eigen::VectorXd ev) ;
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints) ;
    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> oldjoints) ;
    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> minrange, std::vector<double> maxrange) ;

    virtual bool IsSingular(RCS::Pose  pose, double threshold) ;
    virtual void Init(ros::NodeHandle &nh) ;
    void VerifyLimits(std::vector<double> joints) ;
    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            std::vector<double> &min,
            std::vector<double> &max);

};


  class FanucLRMate200idFastKinematics : public IKinematics {
    static double SIGN(double x) {
        return ( x >= 0.0f) ? +1.0f : -1.0f;
    }

    static double NORM(double a, double b, double c, double d) {
        return sqrt(a * a + b * b + c * c + d * d);
    }
    // Convert rotation matrix to quaternion (Daisuke Miyazaki)
    // http://pastebin.com/NikwbL3k

    static RCS::Rotation Convert2Rotation(double *eerot) ;
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    // Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp
    static void Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot);
    double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;
  
public:
    virtual RCS::Pose FK(std::vector<double> joints);
    virtual size_t AllPoseToJoints(RCS::Pose & pose, 
    std::vector<std::vector<double>> &joints) ;
    Eigen::VectorXd ConvertJoints(std::vector<double> v) ;
    std::vector<double> ConvertJoints(Eigen::VectorXd ev) ;
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints) ;
    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> oldjoints) ;
    virtual std::vector<double> IK(RCS::Pose  pose,
            std::vector<double> minrange, std::vector<double> maxrange) ;

    virtual bool IsSingular(RCS::Pose  pose, double threshold) ;
    virtual void Init(ros::NodeHandle &nh) ;
    void VerifyLimits(std::vector<double> joints) ;
    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            std::vector<double> &min,
            std::vector<double> &max);
};      

#if 0
class MotomanSia20d {
public:
    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_vector_;
    std::vector<double> joint_max_vector_;
    std::vector<bool> joint_has_limits_vector_;
    std::vector<std::string> link_names_;
    size_t num_joints_;
    std::vector<int> free_params_;
    bool active_; // Internal variable that indicates whether solvers are configured and ready

    const std::vector<std::string>& getJointNames() const {
        return joint_names_;
    }

    const std::vector<std::string>& getLinkNames() const {
        return link_names_;
    }

    bool initialize(const std::string &robot_description,
            const std::string& group_name,
            const std::string& base_name,
            const std::string& tip_name,
            double search_discretization);

    double harmonize(const std::vector<double> &ik_seed_state,
            std::vector<double> &solution) const;
    
    void getClosestSolution(const IkSolutionList<double> &solutions,
            const std::vector<double> &ik_seed_state,
            std::vector<double> &solution) const;
    
    void fillFreeParams(int count, int *array);

    int GetNumFreeParameters() {
        return 1;
    }

    int* GetFreeParameters() {
        static int freeparams[] = {4};
        return freeparams;
    }

    int GetNumJoints() {
        return 7;
    }

    int GetIkRealSize() {
        return sizeof (duble);
    }
};

#endif