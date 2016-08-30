// David Lu of WU Source Code bsd license blah blah
#pragma once

// Eventually want documentation like: http://wiki.ros.org/katana

/** Private params
	<node pkg="fire_department" type="Firefighter" name="A">
	<!-- set a private parameter for the node -->
	<param name="output_file_name" value="$(node_name)_output_file_name" />
	</node>

	Accessing private parameters is done differently depending on whether you're using the "handle" or "bare" interfaces. In the handle interface you must create a new ros::NodeHandle with the private namespace as its namespace:

	Toggle line numbers
	1 ros::NodeHandle nh("~");
	2 std::string param;
	3 nh.getParam("private_name", param);
	In the bare interface you can access private parameters with the same notation used to describe them, e.g.:

	Toggle line numbers
	1 std::string param;
	2 ros::param::get("~private_name", param);

*/
//http://packages.ros.org/ros/ubuntu/pool/main/r/
#include <cstring>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
// http://wiki.ros.org/moveit_msgs 
// Example client: http://wiki.ros.org/pr2_kinematics/Tutorials/Tutorial%203


#if 1
//http://docs.ros.org/jade/api/moveit_msgs/html/index-msg.html
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#else

#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#endif
#include <urdf/model.h>
#include "nist_fanuc/Globals.h"
using std::string;

//http://wiki.ros.org/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20KDL


class Kinematics {
    public:
        Kinematics();
        bool init(ros::NodeHandle &nh);
        KDL::JntArray joint_min, joint_max;

    private:
       // ros::NodeHandle nh; //, nh_private;
        std::string root_name, tip_name;
        //KDL::JntArray joint_min, joint_max;
        KDL::Chain chain;
        unsigned int num_joints;

        KDL::ChainFkSolverPos_recursive* fk_solver;
        KDL::ChainIkSolverPos_NR_JL *ik_solver_pos;
        KDL::ChainIkSolverVel_pinv* ik_solver_vel;

        ros::ServiceServer ik_service,ik_solver_info_service;
        ros::ServiceServer fk_service,fk_solver_info_service;

        tf::TransformListener tf_listener;

        moveit_msgs::KinematicSolverInfo info;

        bool loadModel(const std::string xml);
        bool readJoints(urdf::Model &robot_model);
        int getJointIndex(const std::string &name);
        int getKDLSegmentIndex(const std::string &name);
public:
        /**
         * @brief This is the basic IK service method that will compute and return an IK solution.
         * @param A request message. See service definition for GetPositionIK for more information on this message.
         * @param The response message. See service definition for GetPositionIK for more information on this message.
         */
        bool getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                           moveit_msgs::GetPositionIK::Response &response);

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        bool getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                             moveit_msgs::GetKinematicSolverInfo::Response &response);

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        bool getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                             moveit_msgs::GetKinematicSolverInfo::Response &response);

        /**
         * @brief This is the basic forward kinematics service that will return information about the kinematics node.
         * @param A request message. See service definition for GetPositionFK for more information on this message.
         * @param The response message. See service definition for GetPositionFK for more information on this message.
         */
        bool getPositionFK(moveit_msgs::GetPositionFK::Request &request,
                           moveit_msgs::GetPositionFK::Response &response);
};


