#ifndef GMOVE
#define GMOVE

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

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

// For debugging output to std::cout in DEBUG 
//#define GODEBUG

namespace gomotion {
    typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

    struct GoMotionParams {
        GoMotionParams(){}
        GoMotionParams(double _vel, double _acc, double _jerk) : 
        vel(_vel), acc(_acc),jerk(_jerk)
        {
        
        }
        double vel; /*< max vel for each motion */
        double acc; /*< max accel for each motion */
        double jerk; /*< max jerk for each motion */
    };

    struct go_motion_interface;

    class GoMotion {
    public:
        GoMotion();
        int Init(JointState here, double deltat);
        int InitPose(tf::Pose here,
                tf::Pose there,
                gomotion::GoMotionParams tparams,
                gomotion::GoMotionParams rparams);
        int InitPose(tf::Pose here, tf::Pose there,
                double seconds);
        int InitJoints(JointState here,
                JointState there,
                std::vector<gomotion::GoMotionParams> params,
                bool bCoordinated=true);
        int InitJoints(JointState here, JointState there,
                double seconds, bool bCoordinated=true);

        void AppendPose(tf::Pose);
        void AppendJoints(JointState);
        tf::Pose NextPose();
        JointState NextJoints();
        bool IsDone();
        void InitStop(); // Then use next pose or net joints
    protected:
        boost::shared_ptr<go_motion_interface> pgm;
        size_t num_joints;
        int InitJoints(JointState here,
                JointState there,
                double seconds,
                std::vector<gomotion::GoMotionParams> jparams,
                bool bCoordinated);
        int InitPose(tf::Pose here, tf::Pose there, double seconds,
                gomotion::GoMotionParams tparams,
                gomotion::GoMotionParams rparams);

    };
};


#endif
