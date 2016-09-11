


// Must webs:
// http://docs.ros.org/kinetic/api/tf_conversions/html/c++/tf__eigen_8cpp_source.html

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include "urdf_model/tfmath.h"

#include "geometry_msgs/PoseStamped.h"
#include "RCS.h"

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace Conversion {

    inline Eigen::Vector3d vectorTFToEigen(tf::Vector3 t) {
        Eigen::Vector3d e;
        e(0) = t[0];
        e(1) = t[1];
        e(2) = t[2];
        return e;
    }

    template<typename T>
    inline tf::Vector3 vectorEigenToTF(T e) { // Eigen::Vector3d e) {
        tf::Vector3 t;
        t[0] = e(0);
        t[1] = e(1);
        t[2] = e(2);
        return t;
    }
   
    
  
    inline geometry_msgs::Pose& TfPose2GeometryPose(RCS::Pose & m, geometry_msgs::Pose & p) {
        p.position.x = m.getOrigin().x();
        p.position.y = m.getOrigin().y();
        p.position.z = m.getOrigin().z();
        p.orientation.x = m.getRotation().x();
        p.orientation.y = m.getRotation().y();
        p.orientation.z = m.getRotation().z();
        p.orientation.w = m.getRotation().w();
        return p;
    }

    inline RCS::Pose & GeometryPose2TfPose(const geometry_msgs::Pose & m, RCS::Pose & p) {
        RCS::Vector3 trans(m.position.x, m.position.y, m.position.z);
        p.setOrigin(trans);
        RCS::Rotation q(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
        p.setRotation(q);
        return p;
    }

     inline RCS::Pose GeomMsgPose2RcsPose(const geometry_msgs::Pose &m) {
        RCS::Pose p;
        RCS::Vector3 trans(m.position.x, m.position.y, m.position.z);
        p.setOrigin(trans);
        RCS::Rotation q(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
        p.setRotation(q);
        return p;
    }
    geometry_msgs::Pose RcsPose2GeomMsgPose(const RCS::Pose &m);
    Eigen::Affine3d GeomMsgPose2Affine3d(const geometry_msgs::Pose &m);
    geometry_msgs::Pose PoseAffineToGeomMsg(const Eigen::Affine3d &e);
    RCS::Pose Affine3d2RcsPose(const Eigen::Affine3d &pose);
    Eigen::Affine3d RcsPose2Affine3d(const RCS::Pose &pose);

    std::vector<geometry_msgs::Pose> RcsPoses2PoseMsgs(const std::vector<RCS::Pose> &src);
    std::vector<RCS::Pose> PoseMsgs2RcsPoses(const std::vector<geometry_msgs::Pose> &src);
    std::vector<Eigen::Affine3d> PoseMsgs2AffEigenPoses(const std::vector<geometry_msgs::Pose> &src);
    std::vector<geometry_msgs::Pose> AffEigenPoses2PoseMsgs(const std::vector<Eigen::Affine3d> &src);
    //JointState     Vector2JointState(const  std::vector<double>  &src);

    inline JointState JntPosVector2JointState(const std::vector<double> &src) {
        JointState joints;
        joints.position = src;
        joints.velocity.resize(src.size(), 0.0);
        joints.effort.resize(src.size(), 0.0);
        return joints;
    }

}
