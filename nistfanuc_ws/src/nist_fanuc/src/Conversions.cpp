
/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 

#include "Conversions.h"

// Eigen conversions
//http://docs.ros.org/kinetic/api/eigen_conversions/html/eigen__msg_8cpp_source.html 

namespace Conversion {

    //==================================================
    // Adapted from: https://github.com/davetcoleman/rviz_visual_tools/blob/kinetic-devel/src/rviz_visual_tools.cpp

    geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose) {
        geometry_msgs::Pose local_msg;
        tf::poseEigenToMsg(pose, local_msg);
        return local_msg;
    }

    Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose) {
        Eigen::Affine3d local_;
        tf::poseMsgToEigen(pose, local_);
        return local_;
    }

    Eigen::Affine3d convertPoint32ToPose(const geometry_msgs::Point32 &point) {
        Eigen::Affine3d shared_pose_eigen_;
        shared_pose_eigen_ = Eigen::Affine3d::Identity();
        shared_pose_eigen_.translation().x() = point.x;
        shared_pose_eigen_.translation().y() = point.y;
        shared_pose_eigen_.translation().z() = point.z;
        return shared_pose_eigen_;
    }

    geometry_msgs::Pose convertPointToPose(const geometry_msgs::Point &point) {
        geometry_msgs::Pose shared_pose_msg_;
        shared_pose_msg_.orientation.x = 0.0;
        shared_pose_msg_.orientation.y = 0.0;
        shared_pose_msg_.orientation.z = 0.0;
        shared_pose_msg_.orientation.w = 1.0;
        shared_pose_msg_.position = point;
        return shared_pose_msg_;
    }

    Eigen::Affine3d convertPointToPose(const Eigen::Vector3d &point) {
        Eigen::Affine3d shared_pose_eigen_;
        shared_pose_eigen_ = Eigen::Affine3d::Identity();
        shared_pose_eigen_.translation() = point;
        return shared_pose_eigen_;
    }

    geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose) {
        geometry_msgs::Pose shared_pose_msg_;
        tf::poseEigenToMsg(pose, shared_pose_msg_);
        return shared_pose_msg_.position;
    }

    Eigen::Vector3d convertPoint(const geometry_msgs::Point &point) {
        Eigen::Vector3d shared_point_eigen_;
        shared_point_eigen_[0] = point.x;
        shared_point_eigen_[1] = point.y;
        shared_point_eigen_[2] = point.z;
        return shared_point_eigen_;
    }

    Eigen::Vector3d convertPoint32(const geometry_msgs::Point32 &point) {
        Eigen::Vector3d shared_point_eigen_;
        shared_point_eigen_[0] = point.x;
        shared_point_eigen_[1] = point.y;
        shared_point_eigen_[2] = point.z;
        return shared_point_eigen_;
    }

    Eigen::Affine3d GeomMsgPose2Affine3d(const geometry_msgs::Pose &m) {
        Eigen::Affine3d e = Eigen::Translation3d(m.position.x,
                m.position.y,
                m.position.z) *
                Eigen::Quaterniond(m.orientation.w,
                m.orientation.x,
                m.orientation.y,
                m.orientation.z);
        return e;
    }

    /*!
     * \brief urdfPose2Affine3d converts urdf pose into an  Eigen affine 4x4 matrix  o represent the pose
     * \param pose is the urdf pose with position and rotation.
     * \return   eigen Affine3d pose 
     */
    Eigen::Affine3d RcsPose2Affine3d(const tf::Pose &pose) {
        Eigen::Quaterniond q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
        Eigen::Affine3d af(Eigen::Translation3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()) * q.toRotationMatrix());
        return af;
    }
    ////////////////////////////////////////////////////////////////////////////

    geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d &point) {
        geometry_msgs::Point32 shared_point32_msg_;
        shared_point32_msg_.x = point[0];
        shared_point32_msg_.y = point[1];
        shared_point32_msg_.z = point[2];
        return shared_point32_msg_;
    }

    geometry_msgs::Point convertPoint(const geometry_msgs::Vector3 &point) {
        geometry_msgs::Point shared_point_msg_;
        shared_point_msg_.x = point.x;
        shared_point_msg_.y = point.y;
        shared_point_msg_.z = point.z;
        return shared_point_msg_;
    }

    geometry_msgs::Point convertPoint(const Eigen::Vector3d &point) {
        geometry_msgs::Point shared_point_msg_;
        shared_point_msg_.x = point.x();
        shared_point_msg_.y = point.y();
        shared_point_msg_.z = point.z();
        return shared_point_msg_;
    }

    geometry_msgs::Pose RcsPose2GeomMsgPose(const tf::Pose &m) {
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

    geometry_msgs::Pose& TfPose2GeometryPose(tf::Pose & m, geometry_msgs::Pose & p) {
        p.position.x = m.getOrigin().x();
        p.position.y = m.getOrigin().y();
        p.position.z = m.getOrigin().z();
        p.orientation.x = m.getRotation().x();
        p.orientation.y = m.getRotation().y();
        p.orientation.z = m.getRotation().z();
        p.orientation.w = m.getRotation().w();
        return p;
    }

    geometry_msgs::Pose PoseAffineToGeomMsg(const Eigen::Affine3d &e) {
        geometry_msgs::Pose m;
        m.position.x = e.translation().x();
        m.position.y = e.translation().y();
        m.position.z = e.translation().z();
        // This is a column major vs row major matrice faux pas!
#if 0
        MatrixEXd em = e.rotation();

        Eigen::Quaterniond q = EMatrix2Quaternion(em);
#endif
        Eigen::Quaterniond q(e.rotation());
        m.orientation.x = q.x();
        m.orientation.y = q.y();
        m.orientation.z = q.z();
        m.orientation.w = q.w();
#if 0
        if (m.orientation.w < 0) {
            m.orientation.x *= -1;
            m.orientation.y *= -1;
            m.orientation.z *= -1;
            m.orientation.w *= -1;
        }
#endif
    }

    /*!
     * \brief affine3d2RcsPose converts an  Eigen affine 4x4 matrix  o represent the pose into a urdf pose 
     * vparam pose   eigen Affine3d pose 
     * \return   urdf pose with position and rotation.
     */
    tf::Pose Affine3d2RcsPose(const Eigen::Affine3d &pose) {
        tf::Pose p;
        p.getOrigin().setX(pose.translation().x());
        p.getOrigin().setY(pose.translation().y());
        p.getOrigin().setZ(pose.translation().z());

        Eigen::Quaterniond q(pose.rotation());
        tf::Quaternion qtf(q.x(), q.y(), q.z(), q.w());
        //std::cout <<  "Affine3d2RcsPose Quaterion = \n" << q.x() << ":" << q.y() << ":" << q.z() << ":" << q.w() << std::endl;

        p.setRotation(qtf);
        //std::cout <<  "After Affine3d2RcsPose Quaterion = \n" << p.getRotation().x() << ":" << p.getRotation().y() << ":" << p.getRotation().z() << ":" << p.getRotation().w() << std::endl;

#if 0
        MatrixEXd m = pose.rotation();
        Eigen::Quaterniond q = EMatrix2Quaterion(m);

        Eigen::Quaterniond q(pose.rotation());
        p.getRotation().setX(q.x());
        p.getRotation().setY(q.y());
        p.getRotation().setZ(q.z());
        p.getRotation().setW(q.w());
#endif
        return p;
    }

    //------------------------------------------------
    // vector conversions

    std::vector<geometry_msgs::Pose> RcsPoses2PoseMsgs(const std::vector<tf::Pose> &src) {
        std::vector<geometry_msgs::Pose> dest;
        dest.resize(src.size());
        std::transform(src.begin(), src.end(), dest.begin(), Conversion::RcsPose2GeomMsgPose);
    }

    std::vector<tf::Pose> PoseMsgs2RcsPoses(const std::vector<geometry_msgs::Pose> &src) {
        std::vector<tf::Pose> dest;
        dest.resize(src.size());
        std::transform(src.begin(), src.end(), dest.begin(), Conversion::GeomMsgPose2RcsPose);
    }

    std::vector<Eigen::Affine3d> PoseMsgs2AffEigenPoses(const std::vector<geometry_msgs::Pose> &src) {
        std::vector<Eigen::Affine3d> dest;
        dest.resize(src.size());
        std::transform(src.begin(), src.end(), dest.begin(), Conversion::GeomMsgPose2Affine3d);
    }

    std::vector<geometry_msgs::Pose> AffEigenPoses2PoseMsgs(const std::vector<Eigen::Affine3d> &src) {
        std::vector<geometry_msgs::Pose> dest;
        dest.resize(src.size());
        std::transform(src.begin(), src.end(), dest.begin(), Conversion::PoseAffineToGeomMsg);
    }
}