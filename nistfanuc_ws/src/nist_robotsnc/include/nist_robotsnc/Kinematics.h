

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
#include <sstream>

#include "RCS.h"
#include "Globals.h"
#include "Conversions.h"
#include "Debug.h"


#define _USE_MATH_DEFINES
#include <math.h>

namespace RCS {
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

    static const int ELBOW_NORMAL = 0x08;
    static const int ELBOW_DOWN = 0x08;
    static const int ELBOW_UP = 0x080;
    static const int ELBOW_FLIPPED = 0x080;

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
     */
    virtual void Configure(int config) = 0;
    virtual std::vector<double> Seed(int config = -1) = 0;

};

class FanucLRmate200iD : public IArmConfiguration {
public:

    FanucLRmate200iD() {
        _size = 6;
        Configure(SHOULDER_DOWN | FOREARM_UP | ELBOW_DOWN | WRIST_NORMAL);

    }
    virtual void Configure(int config);
    virtual std::vector<double> Seed(int config = -1);

};

/**
 * \brief The IKinematics provides is an abstract class with pure virtual functions that are
 * overriden by actual kinematic implementations.
 * 
 * */
class IKinematics {
protected:
    // URDF Derived knowledge
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::vector< double> jointvalues;
    std::vector< double> joint_min;
    std::vector< double> joint_max;
    std::vector< bool> joint_has_limits;
    std::vector< double> joint_effort;
    std::vector< double> joint_velmax;

    std::vector<Eigen::Vector3d> axis;
    std::vector<Eigen::Vector3d> xyzorigin;
    std::vector<Eigen::Vector3d> rpyorigin;


    Eigen::Matrix4d ComputeUrdfTransform(double angle,
            Eigen::Vector3d axis,
            Eigen::Vector3d origin,
            Eigen::Vector3d rotation);


    size_t num_joints;
    std::vector< double> hint;
    std::string _groupname;
    std::string _tiplinkname;
    std::string _rootlinkname;
    std::string _prefix;
    boost::shared_ptr<RCS::CController> _nc;
    bool ParseURDF(std::string xml_string, std::string base_frame);

    std::vector< double> joint_emin;
    std::vector< double> joint_emax;
public:
    static double _testspacing;
    static double _testoffset;
    static double _testepsilon;

    bool IncrementExercise(std::vector<double>& jts);
public:

    IKinematics() {
        _testspacing = 0.5;
        _testoffset = 0.1;
        _testepsilon = 0.1;
    }
    std::string DumpUrdfJoint();

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

    virtual std::vector<double> JointVelMax() {
        return joint_velmax;
    }

    void VerifyLimits(std::vector<double> joints) {
    }
    void ENormalize(double min, double max);

    template<typename OP>
    std::vector<size_t> VerifyKinematics(OP op) {
        std::vector<size_t> scorecard(3, 0);
        std::vector<double> jts(num_joints);

        for (size_t j = 0; j < num_joints; j++) {
            jts[j] = joint_emin[j] + _testoffset;
        }
        bool bFlag = false; // while not done
        while (!bFlag) {
            //std::reverse(jts.begin(), jts.end());
            tf::Pose pose = FK(jts);
            //std::vector<tf::Pose> poses= ComputeAllFk(jts);

            std::vector<double> echojts;
            std::stringstream tmpofs;
            try {
                //                Globals.AssignOfs((std::ostream *) &ofsIkFast,
                //                        (std::ostream *) &tmpofs);
                echojts = IK(pose, jts);
            } catch (...) {
            }


            double err = 0.0;
            int flag = 1;
            if (echojts.size() != jts.size()) {
                flag = -1; // no solution
                scorecard[2]++;

            } else {
                for (size_t k = 0; k < jts.size(); k++)
                    err += fabs(echojts[k] - jts[k]);
                if (err < _testepsilon) {
                    flag = 1; // Green marker
                    scorecard[0]++;

                } else {
                    flag = 0; // Red marker
                    scorecard[1]++;
                }
            }
            if (flag <= 0) {
                //std::cout << RCS::VectorDump(jts).c_str() << "\n";
                ofsRobotExercise << "==========================================================\n";
                ofsRobotExercise << "I0n Joints     =" << RCS::VectorDump<double>(jts).c_str() << "\n";
                ofsRobotExercise << "FK            =" << RCS::DumpPoseSimple(pose).c_str() << "\n";
                //            ofsRobotExercise << "All FK        =" << RCS::DumpPoseSimple(poses[num_joints-1]).c_str() << "\n";
                ofsRobotExercise << "Out Joints    =" << RCS::VectorDump<double>(echojts).c_str() << "\n";
                ofsRobotExercise << tmpofs.str();
            }
            op(flag, pose);
            //std::reverse(jts.begin(), jts.end());
            bFlag = IncrementExercise(jts);
        }
        return scorecard;
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
     * \brief ComputeAllFk performs the forward kinematics using the joint values of the robot provided 
     * and provides each joint position from 0..n.
     * \param vector of all robot joint values in doubles.
     * \return vector of Cartesian robot pose of each joint.
     */
    virtual std::vector<tf::Pose> ComputeAllFk(std::vector<double> thetas);
    /*!
     * \brief FK performs the forward kinematics using the joint values of the robot provided.
     * \param vector of all robot joint values in doubles.
     * \return corresponding Cartesian robot pose of end  effector.
     */
    virtual tf::Pose FK(std::vector<double> jv) = 0;
    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  optional seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual std::vector<double> IK(tf::Pose pose,
            std::vector<double> oldjoints) = 0;


    /*!
     * \brief AllIK solves  the inverse kinematics to find all solutions using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  vector of double vectos to hold all the IK joint solutions.
     * \return number of solutions found.
     */
    virtual size_t AllIK(tf::Pose & pose,
            std::vector<std::vector<double> > & newjoints) = 0;
    /*!
     * \brief NearestJoints finds the joint set that is closest to the old joints.
     * \param  old seed  joint values to use as best guess for IK joint values.
     * \param  vector of double vectos that holds all the IK joint solutions.
     * \return vector of doubles with closest set to seed joints.
     */
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) { return std::vector<double> (); }

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
    virtual bool IsSingular(tf::Pose pose, double threshold) {
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
            joints.position[n] = njoints.position[i]; // joint numbers already adjusted from CRCL to rcs model
            joints.velocity[n] = njoints.velocity[i];
            joints.effort[n] = njoints.effort[i];
            joints.effort[n] = 0.0;
        }
        return joints;
    }

    /*!
     * \brief Compares array of joint positions  against joint minimums and maximums.
     * \param  joints array that contains the value of each joints.
     *  \ param outofbounds array that will contain the indexes of the out of bound joints. 
     * Negative indexes indicate joint value less that minimum joint value.
     * \param  msg is a message describing which joints are out of range.
     * \return bool whether joints in bounds, 0=within bounds, >1 joint(s) out of bounds
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

    /*!
     * \brief Compute distance between seed state and solution joint state.
     * First, normalize all solution joint values to (-2pi,+2pi).
     * \param  ik_seed_state contains original joint value
     * \param  solution is candidate joint values.
     * \return distance between joint vectors
     */
    double Harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
        double dist_sqr = 0;
        std::vector<double> ss = ik_seed_state;
        for (size_t i = 0; i < ik_seed_state.size(); ++i) {
#if 0
            while (ss[i] > 2 * M_PI) {
                ss[i] -= 2 * M_PI;
            }
            while (ss[i] < -2 * M_PI) {
                ss[i] += 2 * M_PI;
            }
            while (solution[i] > 2 * M_PI) {
                solution[i] -= 2 * M_PI;
            }
            while (solution[i] < -2 * M_PI) {
                solution[i] += 2 * M_PI;
            }
#endif
            dist_sqr += fabs(ik_seed_state[i] - solution[i]);
        }
        return dist_sqr;
    }

    void GetClosestSolution(const std::vector<std::vector<double>> &solutions,
            const std::vector<double> &ik_seed_state, std::vector<double> &solution) {
        double mindist = DBL_MAX;
        int minindex = -1;
        std::vector<double> sol;

        for (size_t i = 0; i < solutions.size(); ++i) {
            sol = solutions[i];
            double dist = Harmonize(ik_seed_state, sol);
            if (minindex == -1 || dist < mindist) {
                minindex = i;
                mindist = dist;
            }
        }
        if (minindex >= 0) {
            solution = solutions[minindex];
            Harmonize(ik_seed_state, solution);
        }
    }
    std::string DumpTransformMatrices();
};
typedef boost::shared_ptr<IKinematics> IKinematicsSharedPtr;

class MotomanSia20dFastKinematics : public IKinematics {
public:

    MotomanSia20dFastKinematics(boost::shared_ptr<RCS::CController> nc) {
        _nc = nc;
    }

    virtual tf::Pose FK(std::vector<double> joints);

    virtual size_t AllIK(tf::Pose & pose,
            std::vector<std::vector<double>> &joints);
    Eigen::VectorXd ConvertJoints(std::vector<double> v);
    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual std::vector<double> IK(tf::Pose pose,
            std::vector<double> oldjoints);
    virtual bool IsSingular(tf::Pose pose, double threshold);
    //virtual void Init(ros::NodeHandle &nh) ;
    void VerifyLimits(std::vector<double> joints);


};

class FanucLRMate200idFastKinematics : public IKinematics {
public:

    FanucLRMate200idFastKinematics(boost::shared_ptr<RCS::CController> nc) {
        _nc = nc;
    }

    virtual tf::Pose FK(std::vector<double> joints);
    virtual size_t AllIK(tf::Pose & pose,
            std::vector<std::vector<double>> &joints);
    Eigen::VectorXd ConvertJoints(std::vector<double> v);
    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual std::vector<double> IK(tf::Pose pose,
            std::vector<double> oldjoints);

    virtual bool IsSingular(tf::Pose pose, double threshold);
    void VerifyLimits(std::vector<double> joints);
};


namespace gomotion{
    struct GoKin;
};
class MotomanSia20dGoKin  : public IKinematics {
public:

    MotomanSia20dGoKin(boost::shared_ptr<RCS::CController> nc);
    virtual tf::Pose FK(std::vector<double> joints);
    virtual size_t AllIK(tf::Pose & pose,
            std::vector<std::vector<double>> &joints);
    virtual std::vector<double> IK(tf::Pose pose,
            std::vector<double> oldjoints);
    virtual bool IsSingular(tf::Pose pose, double threshold);
    virtual void Init(ros::NodeHandle & nh);
    void VerifyLimits(std::vector<double> joints) {
    }
   boost::shared_ptr<gomotion::GoKin>  _pGoKin;
};

namespace TRAC_IK{
    struct TRAC_IK;
};

class MotomanSia20dTrak_IK  : public IKinematics {
public:

    MotomanSia20dTrak_IK(boost::shared_ptr<RCS::CController> nc);
    virtual tf::Pose FK(std::vector<double> joints);
    virtual size_t AllIK(tf::Pose & pose,
            std::vector<std::vector<double>> &joints);
    virtual std::vector<double> IK(tf::Pose pose,
            std::vector<double> oldjoints);
    virtual bool IsSingular(tf::Pose pose, double threshold);
    virtual void Init(ros::NodeHandle & nh);
    void VerifyLimits(std::vector<double> joints) {
    }
   boost::shared_ptr<TRAC_IK::TRAC_IK>  _pTRAC_IK;
};

