

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include "gotypes.h"
#include "gointerp.h"
#include "gotraj.h"
#include "gomotion.h"

#include "gomove/gomove.h"
#include "gomove/gomath.h"

namespace gomotion {

    struct go_motion_interface {
        int _type;
        double _deltat;
        go_motion_spec _gms;
        std::vector<go_motion_spec> _space;
        go_motion_queue _gmq;
        go_position _position;
        size_t _queuesize;
        static size_t _id;
    };
    size_t go_motion_interface::_id = 0;

    static go_pose ConvertTfPose(tf::Pose pose) {
        go_pose p;
        p.tran.x = pose.getOrigin().x();
        p.tran.y = pose.getOrigin().y();
        p.tran.z = pose.getOrigin().z();
        p.rot.x = pose.getRotation().x();
        p.rot.y = pose.getRotation().y();
        p.rot.z = pose.getRotation().z();
        p.rot.s = pose.getRotation().w();
        return p;
    }

    static tf::Pose ConvertGoPose(go_pose p) {
        tf::Pose pose;
        pose.setOrigin(tf::Vector3(p.tran.x, p.tran.y, p.tran.z));
        pose.setRotation(tf::Quaternion(p.rot.x, p.rot.y, p.rot.z, p.rot.s));
        return pose;
    }

    GoMotion::GoMotion() {
        pgm = boost::shared_ptr<go_motion_interface>(new go_motion_interface());
    }

    int GoMotion::Init(double cycletime) {
        pgm->_queuesize = 4;
        pgm->_deltat = cycletime;
        pgm->_space.resize(pgm->_queuesize);
        if (GO_RESULT_OK != pgm->_gmq.init(&pgm->_space[0], pgm->_queuesize, pgm->_deltat)) {
            return 1;
        }
    }

    tf::Pose GoMotion::NextPose(tf::Pose here, tf::Pose there, double deltat, GoMotionParams params) {
        go_position position;
        pgm->_gms.init();
        pgm->_gms.set_type(GO_MOTION_WORLD);
        pgm->_gms.set_type(GO_MOTION_LINEAR);
        pgm->_gms.set_id(1);

        position.u.pose = ConvertTfPose(here);
        pgm->_gmq.set_here(&position); // position copied	
        position.u.pose = ConvertTfPose(there);
        pgm->_gms.set_end_position(&position);
        pgm->_gmq.interp(&position);
        return ConvertGoPose(position.u.pose);
    }

    JointState GoMotion::NextJoints(JointState here, JointState there, double deltat, GoMotionParams params) {
        JointState nextjoints;
        go_position position;
        size_t num_joints = here.position.size();
        pgm->_gmq.set_joint_number(num_joints);
        pgm->_gms.init();
        pgm->_gms.set_type(GO_MOTION_JOINT);
        std::copy(here.position.begin(), here.position.end(), position.u.joint);
        pgm->_gmq.set_here(&position); // position copied
        std::copy(there.position.begin(), there.position.end(), position.u.joint);
        pgm->_gms.set_end_position(&position);
        pgm->_gmq.interp(&position);
        nextjoints.position = std::vector<double>(&position.u.joint[0], &position.u.joint[ num_joints]);
        nextjoints.velocity.resize(num_joints, 0.0);

    }
};


