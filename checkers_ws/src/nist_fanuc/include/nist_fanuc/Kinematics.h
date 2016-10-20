

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
//#include "arm_kinematics.h"
#include <urdf/model.h>

#include "RCS.h"
#include "Globals.h"
#include "Conversions.h"
#include "Debug.h"


#define _USE_MATH_DEFINES
#include <math.h>

namespace RCS
{
struct CController;
};

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
    std::vector< bool> joint_has_limits;
    std::vector< double> hint;
    size_t num_joints;
    std::string _groupname;
    std::string _tiplinkname;
    std::string _rootlinkname;
    std::string _prefix;
    boost::shared_ptr<RCS::CController> _nc;
    bool ParseURDF(std::string xml_string, std::string base_frame);

public:

    
    std::string getRootLink() {
        return _rootlinkname;
    }

    std::string getTipLink() {
        return _tiplinkname;
    }

    std::string & Prefix() {
        return _prefix;
    }

    size_t NumJoints() {
        assert(joint_names.size() != 0);
        return joint_names.size();
    }

    virtual void SetHint(std::vector< double> hint) {
        this->hint = hint;
    }

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
    virtual std::vector<double> IK(RCS::Pose pose,
            std::vector<double> oldjoints) = 0;

    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  optional seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual std::vector<double> IK(RCS::Pose pose,
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

    std::vector<unsigned long> AllJointNumbers() {

        std::vector<unsigned long> jointnum(NumJoints());
        std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
        return jointnum;
    }

    /*!
     * \brief Returns true if the determinant of the jacobian is near zero. .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */
    virtual bool IsSingular(RCS::Pose pose, double threshold) {
        return false;
    }

    /*!
     * \brief Initialize kinematics using robot_description to fill parameters .
     * \param  nh ros node handle of node.
     */
    virtual void Init(ros::NodeHandle & nh) {
        ROS_DEBUG_NAMED("ikfast", "Reading xml file from parameter server");
        std::string urdf_xml;
        if (!nh.getParam("robot_description", urdf_xml)) {
            ROS_FATAL_NAMED("IKinematics", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
            //return false;

        }
        if (!ParseURDF(urdf_xml, _rootlinkname))
            ROS_FATAL_NAMED("IKinematics", "Could not parse the xml for kinematic solver", _groupname.c_str());
        num_joints = joint_names.size();
        //return false;

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

    /*!
     * \brief From a list of IK joint solutions find closest based on Arm Configuration instance.
     * \param  solutions array that contains the IK joint solutions.
     *  \ param min array defining minimum joint values within desired range. 
     *  \ param max array defining maximum joint values within desired range. 
     * \return  closed joint aray to arm configuration specification.
     */
    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            std::vector<double> &min,
            std::vector<double> &max) {
        return std::vector<double>();
    }

    /*!
     * \brief From a list of IK joint solutions find closest based on Arm Configuration instance.
     * \param  solutions array that contains the IK joint solutions.
     *  \ param config instance of arm configuration specification, e.g., whether elbow up/down, etc. 
     * \return  closed joint aray to arm configuration specification.
     */
    virtual std::vector<double> FindBoundedSolution(std::vector<std::vector<double>> &solutions,
            boost::shared_ptr<IArmConfiguration> config) {
        return std::vector<double>();
    }

    /*!
     * \brief Compares array of joint positions  against joint minimums and maximums.
     * \param  joints array that contains the value of each joints.
     *  \ param outofbounds array that will contain the indexes of the out of bound joints. 
     * Negative indexes indicate joint value less that minimum joint value.
     * \param  msg is a message describing which joints are out of range.
     * \ return bool whether joints in bounds, 0=within bounds, >1 joint(s) out of bounds
     */
    virtual bool CheckJointPositionLimits(std::vector<double> joints, std::vector<int> &outofbounds, std::string &msg) {
        std::stringstream errmsg;
        outofbounds.clear();
        for (size_t i = 0; i < joints.size(); i++) {
            if (joints[i] < joint_min[i] || joints[i] > joint_max[i]) {
                if (joints[i] < joint_min[i]) {
                    outofbounds.push_back(-i);
                    errmsg << joint_names[i] << "exceed minimum\n";
                }
                if (joints[i] > joint_max[i]) {

                    outofbounds.push_back(i);
                    errmsg << joint_names[i] << "exceed maximum\n";
                }

            }
        }
        msg = errmsg.str();
        return outofbounds.size() > 0;
    }
};
typedef boost::shared_ptr<IKinematics> IKinematicsSharedPtr;

class MotomanSia20dFastKinematics : public IKinematics {

    static double SIGN(double x) {
        return ( x >= 0.0f) ? +1.0f : -1.0f;
    }

    static double NORM(double a, double b, double c, double d) {
        return sqrt(a * a + b * b + c * c + d * d);
    }
    // Convert rotation matrix to quaternion (Daisuke Miyazaki)
    // http://pastebin.com/NikwbL3k

    static RCS::Rotation Convert2Rotation(double *eerot);
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    // Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp

    static void Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot);

    double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;

public:

    MotomanSia20dFastKinematics(boost::shared_ptr<RCS::CController> nc){
        _nc=nc;
    }

    virtual RCS::Pose FK(std::vector<double> joints);

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double>> &joints);
    Eigen::VectorXd ConvertJoints(std::vector<double> v);
    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual std::vector<double> IK(RCS::Pose pose,
            std::vector<double> oldjoints);
    virtual std::vector<double> IK(RCS::Pose pose,
            std::vector<double> minrange, std::vector<double> maxrange);

    virtual bool IsSingular(RCS::Pose pose, double threshold);
    //virtual void Init(ros::NodeHandle &nh) ;
    void VerifyLimits(std::vector<double> joints);
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

    static RCS::Rotation Convert2Rotation(double *eerot);
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    // Found at http://kaist-ros-pkg.googlecode.com/svn/trunk/arm_kinematics_tools/src/ikfastdemo/ikfastdemo.cpp
    static void Convert2RotationMatrix(const RCS::Rotation & quat, double *eerot);
    double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;

public:

     FanucLRMate200idFastKinematics(boost::shared_ptr<RCS::CController> nc){
        _nc=nc;
    }

    virtual RCS::Pose FK(std::vector<double> joints);
    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double>> &joints);
    Eigen::VectorXd ConvertJoints(std::vector<double> v);
    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual std::vector<double> IK(RCS::Pose pose,
            std::vector<double> oldjoints);
    virtual std::vector<double> IK(RCS::Pose pose,
            std::vector<double> minrange, std::vector<double> maxrange);

    virtual bool IsSingular(RCS::Pose pose, double threshold);
    //virtual void Init(ros::NodeHandle &nh) ;
    void VerifyLimits(std::vector<double> joints);
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