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


namespace gomotion {
    typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

    struct GoMotionParams {
        double vel; /*< max vel for each motion */
        double acc; /*< max accel for each motion */
        double jerk; /*< max jerk for each motion */
    };

    struct go_motion_interface;

    class GoMotion {
        GoMotion();
        int Init(double deltat);
        tf::Pose NextPose(tf::Pose here, tf::Pose there, double deltat, GoMotionParams params);
        JointState NextJoints(JointState here, JointState there, double deltat, GoMotionParams params);
    protected:
        boost::shared_ptr<go_motion_interface> pgm;

    };
};


#endif
