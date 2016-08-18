// Controller.h

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
#include <boost/shared_ptr.hpp>
#include <list>
#include <ros/ros.h>
#include <ros/package.h>


#include "NIST/RCSThreadTemplate.h"
#include "NIST/RCSMsgQueue.h"
#include "NIST/BLogging.h"
#include "nist_fanuc/Gripper.h"
#include "nist_fanuc/RCSInterpreter.h"
#include "nist_fanuc/Trajectory.h"
#include "nist_fanuc/Communication.h"
#include "nist_fanuc/moveit.h"
#include "nist_fanuc/RvizMarker.h"
#include "nist_fanuc/arm_kinematics.h"

#include "nistcrcl/CrclCommandMsg.h"
#include "nistcrcl/CrclStatusMsg.h"

namespace RCS {

    extern boost::mutex cncmutex;

    /**
     * \brief The CController provides a collection for all the relevant controller pieces.
     *  The CController is the main controller class to collect all the references/pointers to instances in the project.
     * A global instance of this class, called "Controller", is created and is used throughout the code to reference various instances of 
     * control objects (e.g., kinematics, joint writer, joint reader, etc.)
     *
     */
    struct CController : public RCS::Thread {
        typedef std::list<RCS::CanonCmd> xml_message_list;

        /*!
         * \brief CController constructor that requires a cycle time for RCS thread timing.
         * \param cycletime  in seconds.
         */
        CController(double cycletime);

        ~CController(void);
        static bool bSimulation; /**< simulation flag - not connected to robot */
//        static RCS::CanonWorldModel wm; /**< the world model of the controller */
        static RCS::CanonWorldModel status; /**< current status of controller */
        static RCS::CanonWorldModel laststatus; /**< last status of controller */
//        static RCS::CMessageQueue<RCS::CanonCmd> cmds; /**< queue of commands interpreted from Crcl messages */
        static RCS::CMessageQueue<nistcrcl::CrclCommandMsg > crclcmds; /**< queue of commands interpreted from Crcl messages */
        static xml_message_list donecmds; /**< list of commands interpreted from Crcl messages that have completed*/
        static RCS::CMessageQueue<RCS::CanonCmd> robotcmds; /**< list of commands to be sent to robot */
 
//		static size_t _NumJoints; /**< number of joints in controller robot - assuming serial link manipulator */
//        static std::vector<std::string> joint_names;
//        static std::vector<std::string> link_names;
        
        /*!
         *\brief Verifies that all the pointer references in the controller have been instantiated (i.e., not null).
         */
        bool Verify();

        /*!
         *\brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
         * cmds queue, interprets into robot command messages.
         */
        virtual int Action();

        /*!
         *\brief Setup routine for the controller..
         */
        void Setup(ros::NodeHandle &nh);

        /*!
         *\brief Creates a comma separated string of current state of robot. (Can use other separator). 
         */
        std::string Dump(std::string separator = ",");

        /*!
         *\brief Creates a header line containing names of comma separated string fields that describes the current state of robot. (Can use other separator). 
         */
        std::string DumpHeader(std::string separator = ",");

        //        NVAR(CrclDelegate, boost::shared_ptr<Crcl::CrclDelegateInterface>, crclinterface);
        VAR(Kinematics, boost::shared_ptr<IKinematics>);
        VAR(TrajectoryModel, boost::shared_ptr<CTrajectory>);
        VAR(JointWriter, boost::shared_ptr<CJointWriter>);
        VAR(MoveitPlanner, boost::shared_ptr<MoveitPlanning>);
        VAR(RvizMarker, boost::shared_ptr<CRvizMarker>)
        VAR(EEPoseReader, boost::shared_ptr<CLinkReader>)
        VAR(Gripper, boost::shared_ptr<GripperInterface>)
        ros::Publisher crcl_status; /**< ros publisher information used for crcl status updates */
        ros::Subscriber crcl_cmd; /**< ros subscriber information used for crcl command updates */
        void CmdCallback(const nistcrcl::CrclCommandMsg::ConstPtr& cmdmsg);
        ros::NodeHandle *_nh;
		void MotionLogging();
        //boost::shared_ptr<::Kinematics> armkin;
        /*!
         *\brief Routine to set the kinematics reference pointer. Uses the interface class IKinematics, but can have any implementation instance. 
         */
        void SetKinematics(boost::shared_ptr<IKinematics> k) {
            Kinematics() = k;
            _interpreter._kinematics = k;
        }

        RCSInterpreter _interpreter; /**<  interprets canon commands into robot commands */

        /**<  current new canon command to interpret*/
        NVAR(NewCC, RCS::CanonCmd, _newcc);

        /**<  last canon command interpreted */
        NVAR(LastCC, RCS::CanonCmd, _lastcc);

        //RCS::CanonCmd _newcc; /**<  current new canon command to interpret*/
        //RCS::CanonCmd _lastcc; /**<  last canon command interpreted */
        RCS::CanonCmd GetLastRobotCommand();

        /*!
         *\brief Get the last joint state, if no motion, last actual joint reading, else last joints on robot motion queue. 
         */
        JointState GetLastJointState();

        /*!
         *\brief Get the last commanded , if no motion, use last actual joint reading to compute FK, 
         * else use last joints on robot motion queue to compute FK. 
         */
        RCS::Pose GetLastCommandedPose();


        std::string lastlogstatus;
        static std::vector<std::string> links;

        enum DebugLevel {
            FATAL = 0, WARNING = 2, INFORM = 4, FULL = 5
        }; // ERROR, DEBUG #defines

        enum DebugType {
            CRCL = 0, RPY
        };
        static unsigned long _debugtype; /**<  output crcl xz rotation or roll,pitch, yaw */
        static unsigned long _debuglevel; /**<  level of debugging, 0 least, 5 most */
        static unsigned long _csvlogFlag;
        //static ALogger CsvLogging; /**< controller status csv logging instance */

        enum MotionPlannerEnum {
            NOPLANNER = 0, MOVEIT, DESCARTES, BASIC, WAYPOINT, GOMOTION
        };
        MotionPlannerEnum eCartesianMotionPlanner; /**< type of cartesian motion to use */
        MotionPlannerEnum eJointMotionPlanner; /**< type of joint motion to use */
    };
    extern CController Controller; /**< global declaration of ONE controller */

    //* The RobotCommands is currently a dummy class. The CController thread 

    /**
     * \brief  The RobotStatus is a thread that reads the status of the robot and updates the world model. 
     * The RobotStatus is a separate thread that reads the robot status using ROS communication mechanisms
     * and updates the controller world model based on these values.
     * Currently, it uses an instance of the class JointReader to read joint values from the controller. 
     * It uses a Kinematics pointer reference to compute the current robot pose 
     * using the forward kinematics (FK) routine.
     * It also uses a CrclDelegate pointer reference to update the status reported by CRCL.
     */
    class RobotStatus : public RCS::Thread {
    public:

        /*!
         * \brief RobotStatus constructor that requires a cycle time for RCS thread timing.
         * \param cycletime  in seconds.
         */
        RobotStatus(double cycletime = DEFAULT_LOOP_CYCLE);

        //       NVAR(CrclDelegate, boost::shared_ptr<Crcl::CrclDelegateInterface>, _crclinterface);
        VAR(JointReader, boost::shared_ptr<CJointReader>);
        VAR(Kinematics, boost::shared_ptr<IKinematics>);

        /*!
         * \brief Action is the main loop in the RCS thread timing.
         * Get latest robot joint readings. Use forward kinematics to get current pose.
         * Then, updates the CRCL world model with the latest readings. 
         * \fixme Should it keep track of the command id also - in theory only one CRCl command at a time.
         */
        virtual int Action();

        /*!
         * \brief method to determine if the instance is valid, i.e., has all reference pointers.
         * \return boolean to signify whether component is valid.
         */
        bool Verify() {
            //assert(CrclDelegate() != NULL);
            assert(JointReader() != NULL);
            assert(Kinematics() != NULL);
        }
    };
}
