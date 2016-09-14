// RCS.h

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

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>       /* isnan, sqrt */
#include <stdarg.h>


#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <nistcrcl/CrclCommandMsg.h>
#include <nistcrcl/CrclStatusMsg.h>
#include <nistcrcl/CrclMaxProfileMsg.h>

#include "Globals.h"

//typedef boost::shared_ptr<urdf::Joint> JointSharedPtr;
//typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfaceSharedPtr;

typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

typedef boost::shared_ptr<JointState> JointStateSharedPtr;

// These are defined as macros somewhere
#undef max
#undef min

#define LENGTHUNITS 1000
#define EPSILON                    1E-04
#define DEFAULT_LOOP_CYCLE         0.10

#define DEFAULT_CART_MAX_ACCEL     200.0/LENGTHUNITS
#define DEFAULT_CART_MAX_VEL       2000.0/LENGTHUNITS
#define DEFAULT_JOINT_MAX_ACCEL    200.0/LENGTHUNITS
#define DEFAULT_JOINT_MAX_VEL      2000.0/LENGTHUNITS

#ifndef HAVE_SINCOS
#define HAVE_SINCOS

inline void sincos(double x, double *sx, double *cx) {
    *sx = sin(x);
    *cx = cos(x);
}
#endif

#ifndef Deg2Rad
#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
#endif


namespace RCS {

    //typedef tf::Transform Pose;
    typedef tf::Pose Pose;
    typedef tf::Vector3 Position;
    typedef tf::Quaternion Rotation;
    typedef tf::Vector3 Vector3;
    typedef double Length;
    typedef double LinearVelocity;
    typedef double AngularVelocity;
    typedef std::vector<double> robotAxes;
    void getRPY(const RCS::Pose pose, double &roll, double &pitch, double &yaw);

    struct PoseTolerance : public std::vector<double> {

        enum Type {
            POSITION = 0, ORIENTATION = 1, JOINT = 2
        };

        PoseTolerance() {
            this->resize(3, 0.0);
        } // position, orientation
    };

    inline JointState EmptyJointState(size_t n) {
        JointState js;
        js.position.resize(n, 0.0);
        js.velocity.resize(n, 0.0);
        js.effort.resize(n, 0.0);
        return js;
    }

    inline JointState Subset(JointState js, size_t n) {
        js.position.resize(n);
        js.velocity.resize(n);
        js.effort.resize(n);
        return js;
    }
    inline std::vector<double> Subset(std::vector<double> position, size_t n) {
        position.resize(n);
        return position;
    }
    inline JointState ZeroJointState(size_t numjoints) {
        JointState joints;
        for (size_t i = 0; i < numjoints; i++) {
            joints.position.push_back(0.0);
            joints.velocity.push_back(DEFAULT_JOINT_MAX_VEL);
            joints.effort.push_back(DEFAULT_JOINT_MAX_ACCEL);
        }
        return joints;
    }
    /*!
     * \brief enumeration of  length units. Conversion into ROS compatible meters.
     */
    struct CanonLengthUnit {
        static const int METER = 0;
        static const int MM = 1;
        static const int INCH = 2;
    };

    /*!
     * \brief enumeration of trajectory pose points.
     */
    struct TrajPointType {
        static const int WAYPOINT = 1;
        static const int GOAL = 2;
    };

    /*!
     * \brief enumeration of  angle units. Conversion into ROS compatible radians.
     */
    struct CanonAngleUnit {
        static const int RADIAN = 0;
        static const int DEGREE = 1;
    };

    /*!
     * \brief enumeration of  force units. 
     */
    struct CanonForceUnit {
        static const int NEWTON = 0;
        static const int POUND = 1;
        static const int OUNCE = 2;
    };

    /*!
     * \brief enumeration of  torque units. 
     */
    struct CanonTorqueUnit {
        static const int NEWTONMETER = 0;
        static const int FOOTPOUND = 2;
    };

    /*!
     * \brief enumeration of  return type from Crcl intepretation. If statusreply, requires status
     * sent to Crcl client.
     */
    struct CanonReturn {
        static const int CANON_REJECT = -2;
        static const int CANON_FAILURE = -1;
        static const int CANON_SUCCESS = 0;
        static const int CANON_STATUSREPLY = 1;
        static const int CANON_MOTION = 2;
        static const int CANON_RUNNING = 3;
    };

    /*!
     * \brief enumeration of   Crcl commands. Many Crcl commands are wm parameter setting
     * and require no motion component.
     */
    struct CanonCmdType {
        /**
         *  uint8 initCanon=1
            uint8 endCanon=2
            uint8 actuatejoints=3
            uint8 moveto=4crclcommand
            uint8 dwell=5
            uint8 message=6
            uint8 moveThroughTo=7
            uint8 setCoordinatedMotion=8
            uint8 stopMotion=9
            uint8 setEndEffector=10
            uint8 openToolChange=11
            uint8 closeToolChanger=12
         */
        static const int CANON_NOOP = 0;
        static const int CANON_INIT_CANON = 1;
        static const int CANON_END_CANON = 2;
        static const int CANON_MOVE_JOINT = 3;
        static const int CANON_MOVE_TO = 4;
        static const int CANON_DWELL = 5;
        static const int CANON_MESSAGE = 6;
        static const int CANON_MOVE_THRU = 7;
        static const int CANON_SET_COORDINATED_MOTION = 8;
        static const int CANON_STOP_MOTION = 9;
        static const int CANON_SET_GRIPPER = 10;
        static const int CANON_OPEN_GRIPPER = 11;
        static const int CANON_CLOSE_GRIPPER = 12;
        static const int CANON_SET_TOLERANCE = 0;
	// Extensions:
        static const int CANON_DRAW_OBJECT = 13;
        static const int CANON_ERASE_OBJECT = 14;
        static const int CANON_SET_GRIPPER_POSE = 15;
        static const int CANON_PICK = 16;
        static const int CANON_PLACE = 17;

        static const int CANON_UNKNOWN = -1;
#if 0
        static const int CANON_SET_MAX_CART_ACC = 0;
        static const int CANON_SET_MAX_CART_SPEED = 0;
        static const int CANON_SET_MAX_JOINT_ACC = 0;
        static const int CANON_SET_MAX_JOINT_SPEED = 0;

#endif
    };

    /*!
     * \brief enumeration of  stopping motion, e.g., estop equivalent to immediate.
     */
    struct CanonStopMotionType {
        static const int UNSET = -1;
        static const int IMMEDIATE = 0;
        static const int FAST = 1;
        static const int NORMAL = 2;
    };

    /*!
     * \brief enumeration of  trajectory acceleration profile.
     */
    struct CanonAccProfile {
        static const int MS_IS_UNSET = 0;
        static const int MS_IS_DONE = 1;
        static const int MS_IS_ACCEL = 2;
        static const int MS_IS_CONST = 3;
        static const int MS_IS_DECEL = 4;
        static const int MS_IS_ESTOPPING = 5;
        static const int MS_IS_PAUSED = 6;
    };

    /*!
     * \brief enumeration of  trajectory motion type, joint or cartesian.
     */
    struct MovementType {
        static const int MOVE_DEFAULT = 0;
        static const int MOVE_CARTESIAN = 1;
        static const int MOVE_JOINT = 2;
    };

    /*!
     * \brief enumeration of controller status types for individual commands. 
     * Note, even though command types are listed, not all used or supported.
     */
    struct CanonStatusType {
        /**
        uint8 done=0
        uint8 error=1
        uint8 working=2
         */
        static const int CANON_DONE = 0;
        static const int CANON_ERROR = 2;
        static const int CANON_WORKING = 3;
        static const int CANON_PAUSED = 2;
        static const int CANON_ABORT = 4;
        static const int CANON_WAITING = 5;
    };

    /**
     * \brief IRate is an interface class for defining the allowed motion rates.
     */
    class IRate {
    public:

        IRate() {
            _maximum_velocity = DEFAULT_CART_MAX_VEL;
            _maximum_accel = DEFAULT_CART_MAX_ACCEL;
            _cycleTime = DEFAULT_LOOP_CYCLE;
            Clear();
        }

        IRate(double maximum_velocity, double maximum_accel, double cycleTime) :
        _maximum_velocity(maximum_velocity), _cycleTime(cycleTime), _maximum_accel(maximum_accel) {
            Clear();
        }

        void SetCurrentMotion(double final_velocity, double current_feedrate, double current_velocity) {
            _final_velocity = final_velocity;
            _current_feedrate = current_feedrate;
            _current_velocity = current_velocity;
        }
        NVAR(FinalVelocity, double, _final_velocity);
        NVAR(CurrentFeedrate, double, _current_feedrate);
        NVAR(CurrentVelocity, double, _current_velocity);
        NVAR(MaximumVelocity, double, _maximum_velocity);
        NVAR(MaximumAccel, double, _maximum_accel);

        VAR(MaxJoinVelocity, double);
        VAR(MaxJointAccel, double);
        VAR(JointMaxLimit, std::vector<double>);
        VAR(JointMinLimit, std::vector<double>);
        VAR(JointVelLimit, std::vector<double>);
        VAR(JointAccLimit, std::vector<double>);

        NVAR(CycleTime, double, _cycleTime);
        NVAR(CurrentAccel, double, _current_accel);
        NVAR(MsFlag, int, _msflag);
    private:

        void Clear() {
            FinalVelocity() = CurrentVelocity() = CurrentAccel() = 0.0;
            MaximumVelocity() = CurrentFeedrate() = DEFAULT_CART_MAX_VEL;
            MaximumAccel() = DEFAULT_CART_MAX_ACCEL;
            MaxJoinVelocity() = DEFAULT_JOINT_MAX_VEL;
            MsFlag() = CanonAccProfile::MS_IS_UNSET;
        }
    };

    /*!
     * \brief CanonCmd is the controller command structure. 
     * 
     * int64 crclcommand
           crclcommandnum
        # https://github.com/ros/common_msgs 
        geometry_msgs/Pose  finalpose
        geometry_msgs/Pose[] waypoints
        # Below joint info could be  trajectory_msgs/JointTrajectoryPoint
        sensor_msgs/JointState joints
        bool bStraight
        float64   dwell_seconds
        string opmessage
        bool bCoordinated
        float64 eepercent
        CrclMaxProfileMsg[] profile # maximum profile 
     */
    struct CanonCmd : public nistcrcl::CrclCommandMsg {

        /*!
         * \brief CanonCmd constructor. 
         */
        CanonCmd() {
            //           Init();
            CommandID() = _cmdid++;
        }

        void Set(nistcrcl::CrclCommandMsg &msg) {
            this->crclcommand = msg.crclcommand;
            this->crclcommandnum = msg.crclcommandnum;
            this->finalpose = msg.finalpose;
            this->waypoints = msg.waypoints;

            this->joints = msg.joints;
            this->jointnum = msg.jointnum;
            this->bStraight = msg.bStraight;

            this->dwell_seconds = msg.dwell_seconds;
            this->opmessage = msg.opmessage;
            this->bCoordinated = msg.bCoordinated;

            this->eepercent = msg.eepercent;
            this->partname = msg.partname;
            this->partfeature = msg.partfeature;
            this->hint = msg.hint;
            this->partcolor = msg.partcolor;
 
	    // Doesn't seem to work copying
            //    static_cast<nistcrcl::CrclCommandMsg>(*this)  = msg;

        }
        void Init();

        VAR(CommandID, unsigned long long);
        VAR(ParentCommandID, unsigned long long);
        VAR(StatusID, unsigned long long);
        VAR(Rates, IRate);
	VAR(ConfigMin, std::vector<double>);
	VAR(ConfigMax, std::vector<double>);

        static unsigned long long _cmdid;

        bool IsMotionCmd();

        //        int &cmd; /**<  command  type */
        int status; /**<  status type */
        int type; /**<  trajectory points  type */
        int stoptype; /**<  stop trajectory choice */
        int accprofile; /**<  current trajectory acceleration profile */
#if 0
        bool bCoordinated; /**<  coordinated joint trajectory motion boolean */
        bool bStraight; /**<  straigth cartesian trajectory motion boolean */
        double dwell; /**<  time for dwelling in seconds */
        //JointState joints; /**<  commanded joint state */
        std::vector<RCS::Pose> waypoints; /**< commanded cartesian waypoints in trajectory */
#endif
        //      double &gripperPos; /**<  gripper position 0 to 1 */
        //       RCS::Pose &pose; /**<  commanded pose state */

        double absTransAcc; /**<  cartesian translational acceleration */
        double absTransSpeed; /**<  cartesian translational velocity */
        double absRotAcc; /**<  cartesian rotation acceleration */
        double absRotSpeed; /**<  cartesian rotation velocity */
        double absJointAcc; /**<  joint max acceleration */
        double absJointSpeed; /**<  joint max velocity */

        std::vector<double> speed; /**<  vector of joint velocities */
        // std::vector<int> jointnum; /**<  vector of joint numbers used by command */

        //JointState seed; /**<  near pose joint state */
        RCS::PoseTolerance endPoseTol; /**<  commanded tolerance */
        // std::vector<RCS::Pose> waypointtolerances; /**< commanded cartesian waypoints in trajectory */
        RCS::PoseTolerance intermediatePtTol; /**< commanded cartesian waypoints in trajectory */
        RCS::PoseTolerance gripperTol; /**< gripper trajectory */

    };

    /*!
     * \brief CanonWorldModel describes the controller state. Includes reference to robot model.
     */
    struct CanonWorldModel {

        /*!
         * \brief CanonWorldModel constructor that initializes parameterization.
         */
        CanonWorldModel() {
        }
        void Init();

        CanonCmd echocmd; /**<  copy of current command */
        int echo_cmd; /**<  copy of current command type */
        int crclcommandstatus;
        int echo_status; /**<  copy of current status type */

        /*!
         * \brief Cycletime of the world model. 
         * /fixme what is this
         */
        double getCycleTime() {
            return _cycleTime; // milliseconds
        }

        //ModelInterfaceSharedPtr robot_model; /**<  pointer to robot model */

        // //////////////////////
        // double maxAccel[3];
        // double maxVel[3];
        double maxTransAccel; /**<  max translation acceleration */
        double maxTransVel; /**<  max translation velocity */
        double maxRotAccel; /**<  max rotational acceleration */
        double maxRotVel; /**<  max rotational velocity */
        double maxJointAccel; /**<  max joint acceleration */
        double maxJointVel; /**<  max joint velocity */
        double _cycleTime; /**<  cycle time */
        JointState currentjoints; /**<  current joint state */
        RCS::Pose currentpose; /**<  current robot pose */
        bool bStraight;
        bool bCoordinated;
        double eepercent;
    };

};



