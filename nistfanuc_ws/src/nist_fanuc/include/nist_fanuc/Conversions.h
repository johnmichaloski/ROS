

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include "urdf_model/tfmath.h"

#include "geometry_msgs/PoseStamped.h"
#include "RCS.h"
namespace Conversion {

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

    RCS::Pose GeomMsgPose2UrdfPose(const geometry_msgs::Pose &m);
    geometry_msgs::Pose UrdfPose2GeomMsgPose(const RCS::Pose &m);
    Eigen::Affine3d GeomMsgPose2Affine3d(const geometry_msgs::Pose &m);
    geometry_msgs::Pose PoseAffineToGeomMsg(const Eigen::Affine3d &e);
    RCS::Pose Affine3d2UrdfPose(const Eigen::Affine3d &pose);
    Eigen::Affine3d UrdfPose2Affine3d(const RCS::Pose &pose);

    std::vector<geometry_msgs::Pose> UrdfPoses2PoseMsgs(const std::vector<RCS::Pose> &src);
    std::vector<RCS::Pose> PoseMsgs2UrdfPoses(const std::vector<geometry_msgs::Pose> &src);
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