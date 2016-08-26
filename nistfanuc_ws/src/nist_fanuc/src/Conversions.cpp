

#include "Conversions.h"
// Eigen conversions
//http://docs.ros.org/kinetic/api/eigen_conversions/html/eigen__msg_8cpp_source.html 

namespace Conversion {

    geometry_msgs::Pose RcsPose2GeomMsgPose(const RCS::Pose &m) {
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
     * \brief urdfPose2Affine3d converts urdf pose into an  Eigen affine 4x4 matrix  o represent the pose
     * \param pose is the urdf pose with position and rotation.
     * \return   eigen Affine3d pose 
     */
    Eigen::Affine3d RcsPose2Affine3d(const RCS::Pose &pose) {
        Eigen::Quaterniond q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
        Eigen::Affine3d af(Eigen::Translation3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()) * q.toRotationMatrix());
        return af;
    }

    /*!
     * \brief affine3d2RcsPose converts an  Eigen affine 4x4 matrix  o represent the pose into a urdf pose 
     * vparam pose   eigen Affine3d pose 
     * \return   urdf pose with position and rotation.
     */
    RCS::Pose Affine3d2RcsPose(const Eigen::Affine3d &pose) {
        RCS::Pose p;
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

    std::vector<geometry_msgs::Pose> RcsPoses2PoseMsgs(const std::vector<RCS::Pose> &src) {
        std::vector<geometry_msgs::Pose> dest;
        dest.resize(src.size());
        std::transform(src.begin(), src.end(), dest.begin(), Conversion::RcsPose2GeomMsgPose);
    }

    std::vector<RCS::Pose> PoseMsgs2RcsPoses(const std::vector<geometry_msgs::Pose> &src) {
        std::vector<RCS::Pose> dest;
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