


// Must webs:
// http://docs.ros.org/kinetic/api/tf_conversions/html/c++/tf__eigen_8cpp_source.html

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

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS messages
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// CRCL representations
#include "RCS.h"

#ifndef Deg2Rad
#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
#endif


// tf documentation found at:
// Transform http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transform.html
// Quaternion http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Quaternion.html

/**
 * \brief The namespace Conversion provides some utilities to convert from one representation into another. 
 * Representations include tf, Eigen::Affine3d, std::vector, geometry_msgs::Pose, 
 * geometry_msgs::Point, and CRCL.
 * 
 * Clearly there may be faster const references, but they require special line to convert,
 * cannot be done in line since you cannot pass a const reference to a constructor on the
 * stack in g++ unless you override a warning.
 * 
 * For g++, compilation would be faster if these conversion routines were placed in source
 * file (cpp) OR you used precompiled header in g++. 
 * here is a "silent" error when exceeding precompiled header limits  in g++. (Or was at one time).
 * 
 * */
namespace Conversion {

    /*!
     * \brief Empty conversion of type from into type to. If called, asserts.
     * \param from is defined in the template corresponding typename.
     * \return to is defined in the template corresponding typename
     */
    template<typename From, typename To>
    inline To Convert(From f) {
        assert(0);
    }

    // tf

    /*!
     * \brief Convert Eigen::Affine3d into tf::Pose.
     * \param pose is copy constructor of Eigen::Affine3d.
     * \return tf::Pose 
     */
    template<>
    inline tf::Pose Convert<Eigen::Affine3d, tf::Pose>(Eigen::Affine3d pose) {
#if 0
        return tf::Pose(tf::Quaternion(pose.rotation().x(), pose.rotation().y(), pose.rotation().z(), pose.rotation().w()),
                tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
#else
        tf::Pose p;
        p.getOrigin().setX(pose.translation().x());
        p.getOrigin().setY(pose.translation().y());
        p.getOrigin().setZ(pose.translation().z());

        Eigen::Quaterniond q(pose.rotation());
        tf::Quaternion qtf(q.x(), q.y(), q.z(), q.w());
        p.setRotation(qtf);
        return p;
#endif
    }

    /*!
     * \brief Convert geometry_msgs::Pose into tf::Pose.
     * \param pose is copy constructor of geometry_msgs::Pose.
     * \return tf::Pose 
     */
    template<>
    inline tf::Pose Convert<geometry_msgs::Pose, tf::Pose>(geometry_msgs::Pose m) {
        return tf::Pose(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                tf::Vector3(m.position.x, m.position.y, m.position.z));
    }

    /*!
     * \brief Convert Eigen::Quaterniond into tf::Quaternion.
     * \param e is copy constructor of Eigen::Quaterniond.
     * \return tf::Quaternion 
     */
    template<>
    inline tf::Quaternion Convert<Eigen::Quaterniond, tf::Quaternion> (Eigen::Quaterniond e) {
        return tf::Quaternion(e.x(), e.y(), e.z(), e.w());
    }

    /*!
     * \brief Convert std::vector<double> into tf::Pose.
     * \param ds are an array of 7 doubles to define tf Pose.
     * \return tf::Pose 
     */
    template<>
    inline tf::Pose Convert<std::vector<double>, tf::Pose>(std::vector<double> ds) {
        assert(ds.size() > 6);
        return tf::Pose(tf::Quaternion(ds[3], ds[4], ds[5], ds[6]), tf::Vector3(ds[0], ds[1], ds[2]));
    }

    /*!
     * \brief Convert std::vector<double> into tf::Vector3.
     * \param ds are an array of 3 doubles to define tf Vector3.
     * \return tf::Vector3 
     */
    template<>
    inline tf::Vector3 Convert<std::vector<double>, tf::Vector3>(std::vector<double> ds) {
        assert(ds.size() > 2);
        return tf::Vector3(ds[0], ds[1], ds[2]);
    }

    /*!
     * \brief Convert tf::Quaternion into tf::Pose.
     * \param q rotation is converted into a tf::Pose.
     * \return tf::Pose 
     */
    template<>
    inline tf::Pose Convert<tf::Quaternion, tf::Pose> (tf::Quaternion q) {
        return tf::Pose(q, tf::Vector3(0., 0., 0.));
    }

    /*!
     * \brief Convert tf::Vector3 into tf::Pose.
     * \param a t translation is converted into a tf::Pose.
     * \return tf::Pose 
     */
    template<>
    inline tf::Pose Convert<tf::Vector3, tf::Pose>(tf::Vector3 t) {
        return tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), t);
    }

    /*!
     * \brief CreateRPYPose taks array of double and create a tf::Pose.
     * \param a std array of 6 doubles to create pose (rpy + xyz).
     * \return tf::Pose 
     */
    inline tf::Pose CreateRPYPose(std::vector<double> ds) {
        assert(ds.size() > 5);
        return tf::Pose(tf::Quaternion(ds[3], ds[4], ds[5]), tf::Vector3(ds[0], ds[1], ds[2]));
    }

    /*!
     * \brief Create Pose from a axis and angle rotation representation.
     * \param axis is the unit vector to rotation around.
     * \param angle is the angle of rotation in radians.
     * \return tf::Pose 
     */
    inline tf::Pose CreatePose(tf::Vector3 axis, double angle) {
        return tf::Pose(tf::Quaternion(axis, angle), tf::Vector3(0.0, 0.0, 0.0));
    }

    /*!
     * \brief Create Pose from a rpy rotation representation designated in radians.
     * \param roll rotation around x axis in radians.
     * \param pitch rotation around y axis in radians.
     * \param yaw rotation around z axis in radians.
     * \return tf::Pose 
     */
    inline tf::Quaternion RPYRadians(double roll, double pitch, double yaw) {
        return tf::Quaternion(yaw, pitch, roll);
    }

    /*!
     * \brief Create Pose from a rpy rotation representation designated in degrees.
     * \param roll rotation around x axis in degrees.
     * \param pitch rotation around y axis in degrees.
     * \param yaw rotation around z axis in degrees.
     * \return tf::Pose 
     */
    inline tf::Quaternion RPYDegrees(double roll, double pitch, double yaw) {
        return tf::Quaternion(Deg2Rad(yaw), Deg2Rad(pitch), Deg2Rad(roll));
    }

    template<typename T>
    inline tf::Vector3 vectorEigenToTFVector(T e) {
        return tf::Vector3(e(0), e(1), e(2));
    }

    template<typename T>
    inline tf::Pose vectorEigenToTF(T e) {
        return tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(e(0), e(1), e(2)));
    }

    template<typename T>
    inline tf::Vector3 matrixEigenToTfVector(T e) {
        return tf::Vector3(e(0, 3), e(1, 3), e(2, 3));
    }

    inline tf::Pose Identity() {
        tf::Transform t;
        t.setIdentity();
        return t;
    }

    // Eigen

    /*!
     * \brief Convert<tf::Pose, Eigen::Affine3d> converts tf pose into an  Eigen affine 4x4 matrix  o represent the pose
     * \param pose is the tf pose with position and orientation.
     * \return   Eigen Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<tf::Pose, Eigen::Affine3d>(tf::Pose pose) {
        Eigen::Quaterniond q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
        return Eigen::Affine3d(Eigen::Translation3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()) * q.toRotationMatrix());
    }

    /*!
     * \brief Convert Eigen::Quaternion into an  Eigen affine 4x4 matrix  o represent the pose
     * \param q is the Eigen::Quaternion orientation.
     * \return   Eigen Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<Eigen::Quaternion<double>, Eigen::Affine3d> (Eigen::Quaternion<double> q) {
        return Eigen::Affine3d::Identity() * q;
    }

    /*!
     * \brief Convert geometry_msgs::Pose into an  Eigen affine3d 4x4 matrix  o represent the pose.
     * Uses tf conversion utilities.
     * \param pose is defined as a geometry_msgs::Pose..
     * \return  Eigen Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<geometry_msgs::Pose, Eigen::Affine3d> (geometry_msgs::Pose pose) {
        Eigen::Affine3d local_;
        tf::poseMsgToEigen(pose, local_);
        return local_;
    }

    /*!
     * \brief Convert geometry_msgs::Pose into an  Eigen::Translation3d.
     * \param pose is defined as a geometry_msgs::Pose..
     * \return  Eigen::Translation3d vector 
     */
    template<>
    inline Eigen::Translation3d Convert<geometry_msgs::Pose, Eigen::Translation3d>(geometry_msgs::Pose pose) {
        return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z);

    }

    /*!
     * \brief Convert tf::Vector3 into an  Eigen::Vector3d.
     * \param translation is defined as a tf::Vector3..
     * \return  Eigen::Vector3d vector 
     */
    template<>
    inline Eigen::Vector3d Convert< tf::Vector3, Eigen::Vector3d>(tf::Vector3 t) {
        return Eigen::Vector3d(t[0], t[1], t[2]);
    }

    /*!
     * \brief Convert Eigen::Vector3d translation into an  Eigen::Affine3d pose.
     * \param translation is defined as a Eigen::Vector3d.
     * \return  Eigen::Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<Eigen::Vector3d, Eigen::Affine3d >(Eigen::Vector3d translation) {
        Eigen::Affine3d shared_pose_eigen_ = Eigen::Affine3d::Identity();
        shared_pose_eigen_.translation() = translation;
        return shared_pose_eigen_;
    }

    /*!
     * \brief Create Eigen::Affine3d as an axis angle definition around z axis.
     * \param zangle is angle of rotation in radians around Z.
     * \return  Eigen::Affine3d pose 
     */
    inline Eigen::Affine3d CreateEigenPose(double zangle) {
        return Eigen::Affine3d::Identity() * Eigen::AngleAxisd(zangle, Eigen::Vector3d::UnitZ());
    }

    /*!
     * \brief Convert Eigen::Translation3d translation into an  Eigen::Affine3d pose.
     * \param translation is defined as a Eigen::Translation3d.
     * \return  Eigen::Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<Eigen::Translation3d, Eigen::Affine3d>(Eigen::Translation3d trans) {
        return Eigen::Affine3d::Identity() * trans;
    }

    /*!
     * \brief Convert geometry_msgs::Point32 translation into an  Eigen::Vector3d vector.
     * \param point is translation  defined as a geometry_msgs::Point32.
     * \return  Eigen::Vector3d vector 
     */
    template<>
    inline Eigen::Vector3d Convert<geometry_msgs::Point32, Eigen::Vector3d>(geometry_msgs::Point32 point) {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    /*!
     * \brief Convert geometry_msgs::Point32 translation into an  Eigen::Affine3d pose.
     * \param point is translation  defined as a geometry_msgs::Point32.
     * \return  Eigen::Affine3d pose 
     */
    template<>
    inline Eigen::Affine3d Convert<geometry_msgs::Point32, Eigen::Affine3d>(geometry_msgs::Point32 point) {
        return Eigen::Affine3d::Identity() * Eigen::Translation3d(point.x, point.y, point.z);
    }

    //Eigen::Affine3d GeomMsgPose2Affine3d(const geometry_msgs::Pose &m);
    //Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose);
    //   Eigen::Affine3d tfPose2Affine3d(const tf::Pose pose);
    //   Eigen::Affine3d poseTFToEigen(const tf::Pose t);


    // geometry_msgs - constructor hell.

    /*!
     * \brief Convert tf::Pose pose into an  geometry_msgs::Pose pose.
     * \param m is a tf::Pose transform matrix.
     * \return  geometry_msgs::Pose pose 
     */
    template<>
    inline geometry_msgs::Pose Convert< tf::Pose, geometry_msgs::Pose> (tf::Pose m) {
        geometry_msgs::Pose p;
        p.position.x = m.getOrigin().x();
        p.position.y = m.getOrigin().y();
        p.position.z = m.getOrigin().z();
        p.orientation.x = m.getRotation().x();
        p.orientation.y = m.getRotation().y();
        p.orientation.z = m.getRotation().z();
        p.orientation.w = m.getRotation().w();
        return p;
    }

    /*!
     * \brief Convert geometry_msgs::Point point into an  geometry_msgs::Pose pose.
     * \param point geometry_msgs::Point is translation.
     * \return  geometry_msgs::Pose pose.
     */
    template<>
    inline geometry_msgs::Pose Convert<geometry_msgs::Point, geometry_msgs::Pose>(geometry_msgs::Point point) {
        geometry_msgs::Pose shared_pose_msg_;
        shared_pose_msg_.orientation.x = 0.0;
        shared_pose_msg_.orientation.y = 0.0;
        shared_pose_msg_.orientation.z = 0.0;
        shared_pose_msg_.orientation.w = 1.0;
        shared_pose_msg_.position = point;
        return shared_pose_msg_;
    }
    //geometry_msgs::Pose convertPointToPose(const geometry_msgs::Point &point);
    geometry_msgs::Point convertPoint(const geometry_msgs::Vector3 &point);
    //geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);
    geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose);
    //geometry_msgs::Point convertPoint(const Eigen::Vector3d &point);
    geometry_msgs::Pose PoseAffineToGeomMsg(const Eigen::Affine3d &e);
    // geometry_msgs constructor nightmare
#if 1
    //    template<>
    //    geometry_msgs::Point Convert<Eigen::Vector3d, geometry_msgs::Point >(Eigen::Vector3d point) {
    //        return geometry_msgs::Point(point[0], point[1], point[2]);
    //    }

    //    template<>
    //    geometry_msgs::Point Convert< geometry_msgs::Point, geometry_msgs::Vector3>(geometry_msgs::Vector3 point) {
    //        return geometry_msgs::Point(point.x, point.y, point.z);
    //     }

    /*!
     * \brief Convert Eigen::Vector3d point into an geometry_msgs::Point position vector.
     * \param point Eigen::Vector3d is translation.
     * \return  geometry_msgs::Point position vector.
     */
    template<>
    inline geometry_msgs::Point Convert<Eigen::Vector3d, geometry_msgs::Point>(Eigen::Vector3d point) {
        geometry_msgs::Point pt;
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        return pt;
    }
#endif

    /*!
     * \brief Convert Eigen::Affine3d pose into an geometry_msgs::Pose pose.
     * \param Eigen::Affine3d is equivalent pose.
     * \return  geometry_msgs::Pose pose.
     */
    template<>
    inline geometry_msgs::Pose Convert <Eigen::Affine3d, geometry_msgs::Pose> (Eigen::Affine3d pose) {
        geometry_msgs::Pose local_msg;
        tf::poseEigenToMsg(pose, local_msg);
        return local_msg;
    }

    /*!
     * \brief Convert Eigen::Affine3d pose into an geometry_msgs::Point translation element.
     * \param Eigen::Affine3d is pose.
     * \return  geometry_msgs::Point translation element.
     */
    template<>
    inline geometry_msgs::Point Convert<Eigen::Affine3d, geometry_msgs::Point> (Eigen::Affine3d pose) {
        geometry_msgs::Pose shared_pose_msg_;
        tf::poseEigenToMsg(pose, shared_pose_msg_);
        return shared_pose_msg_.position;
    }
    // CRCL
    // typdef sensor_msgs::JointState_<std::allocator<void> > JointState;

    /*!
     * \brief Convert array of std::vector<double> doubles into an JointState position, but blanking velcity, and effort.
     * \param Estd::vector<double>> of doubles for each joint.
     * \return  sensor_msgs::JointState_<std::allocator<void> > definition.
     */
    template<>
    inline JointState Convert<std::vector<double>, JointState>(std::vector<double> src) {
        JointState joints;
        joints.position = src;
        joints.velocity.resize(src.size(), 0.0);
        joints.effort.resize(src.size(), 0.0);
        return joints;
    }
}
