


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

#include "RCS.h"

// tf
// Transform http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transform.html
// Quaternion http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Quaternion.html

namespace Conversion {

     // tf
   template<typename T>
    inline tf::Vector3 vectorEigenToTF(T e) {
        return tf::Vector3(e(0), e(1), e(2));
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

    tf::Pose Affine3d2RcsPose(const Eigen::Affine3d &pose);

    inline tf::Pose & GeometryPose2TfPose(const geometry_msgs::Pose & m, tf::Pose & p) {
        p.setOrigin(tf::Vector3(m.position.x, m.position.y, m.position.z));
        p.setRotation(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w));
        return p;
    }

    inline tf::Pose GeomMsgPose2RcsPose(const geometry_msgs::Pose &m) {
        tf::Pose p;
        p.setOrigin(tf::Vector3(m.position.x, m.position.y, m.position.z));
        p.setRotation(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w));
        return p;
    }

    // geometry_msgs
    geometry_msgs::Pose convertPointToPose(const geometry_msgs::Point &point);
    geometry_msgs::Point convertPoint(const geometry_msgs::Vector3 &point);
    geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);
    geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose);
    geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d &point);
    geometry_msgs::Point convertPoint(const Eigen::Vector3d &point);
    geometry_msgs::Pose PoseAffineToGeomMsg(const Eigen::Affine3d &e);
    geometry_msgs::Pose RcsPose2GeomMsgPose(const tf::Pose &m);
    geometry_msgs::Pose& TfPose2GeometryPose(tf::Pose & m, geometry_msgs::Pose & p) ;
    
    // Eigen
    Eigen::Affine3d GeomMsgPose2Affine3d(const geometry_msgs::Pose &m);
    Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose);
    Eigen::Affine3d convertPoint32ToPose(const geometry_msgs::Point32 &point);
    Eigen::Affine3d convertPointToPose(const Eigen::Vector3d &point);
    Eigen::Vector3d convertPoint(const geometry_msgs::Point &point);
    Eigen::Vector3d convertPoint32(const geometry_msgs::Point32 &point);
    Eigen::Affine3d RcsPose2Affine3d(const tf::Pose &pose);

    inline Eigen::Translation3d RcsPose2EigenPosition(const geometry_msgs::Pose &pose) {
        return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z);
    }

    inline Eigen::Affine3d EigenPosition2EigenAffine3d(const Eigen::Translation3d &trans) {
        return Eigen::Affine3d::Identity() * trans;
    }

    inline Eigen::Vector3d vectorTFToEigen(tf::Vector3 t) {
        return Eigen::Vector3d(t[0], t[1], t[2]);
    }


    // CRCL
    inline JointState JntPosVector2JointState(const std::vector<double> &src) {
        JointState joints;
        joints.position = src;
        joints.velocity.resize(src.size(), 0.0);
        joints.effort.resize(src.size(), 0.0);
        return joints;
    }
    
    // Vector conversions
    std::vector<geometry_msgs::Pose> RcsPoses2PoseMsgs(const std::vector<tf::Pose> &src);
    std::vector<tf::Pose> PoseMsgs2RcsPoses(const std::vector<geometry_msgs::Pose> &src);
    std::vector<Eigen::Affine3d> PoseMsgs2AffEigenPoses(const std::vector<geometry_msgs::Pose> &src);
    std::vector<geometry_msgs::Pose> AffEigenPoses2PoseMsgs(const std::vector<Eigen::Affine3d> &src);
    //JointState     Vector2JointState(const  std::vector<double>  &src);

}
