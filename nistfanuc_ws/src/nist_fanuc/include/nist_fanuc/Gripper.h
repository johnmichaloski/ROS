

//
// GripperHw.h
//

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


#include <string>
#include <vector>
#include <math.h>

// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
/***
    // Couple code attempts at reading from robot joints names - see above
    crclinterface->crclwm.jointnames.clear();

    _nh.getParam("controller_joint_names", crclinterface->crclwm.jointnames);
    std::cout << VectorDump<std::string> (crclinterface->crclwm.jointnames);

In launch file, add line:
        <rosparam  command="load" file="$(find pkg)/config/joint_names_lrmate200id.yaml" />

In "..../config/joint_names_lrmate200id.yaml" file:
controller_joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

The ros parameter name is given by the yaml: controller_joint_names
 */

class GripperInterface {
public:
    ros::NodeHandle &_nh;
    ros::Publisher joint_pub;
    std::vector<std::string> joint_names;
    const double degree = M_PI / 180;
    sensor_msgs::JointState joint_state;

    GripperInterface(ros::NodeHandle &nh) :
    _nh(nh) {
        _nh.getParam("controller_joint_names", joint_names);
        if (joint_names.size() == 0) {
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name[0] = "robotiq_85_left_knuckle_joint"; // 0 to 90o
            joint_state.name[1] = "robotiq_85_right_knuckle_joint"; // 0 to 90o
            joint_state.name[2] = "robotiq_85_left_inner_knuckle_joint"; // 0 to 90o
            joint_state.name[3] = "robotiq_85_right_inner_knuckle_joint"; // 0 to 90o
            joint_state.name[4] = "robotiq_85_left_finger_tip_joint"; // 0 to 90o
            joint_state.name[5] = "robotiq_85_right_finger_tip_joint"; // 0 to 90o
        } else {

            joint_state.position.resize(joint_names.size());
        }

    }

    void init() {
        ROS_INFO("GripperHwInterface init");
        try {
            //urdf =  getUrdf(_nh, "robot_description");
            joint_pub = _nh.advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 10);



        } catch (...) {
            ROS_ERROR("GripperInterface Failed to init");

        }
    }

    void close() {
        ROS_INFO("GripperInterface close");

        joint_state.position[0] = 0.500;
        joint_state.position[1] = 0.500;
        joint_state.position[2] = 0.500;
        joint_state.position[3] = 0.500;
        joint_state.position[4] = -0.500;
        joint_state.position[5] = -0.500;
        publish_jointstate();
    }

    void publish_jointstate() {
        joint_state.header.stamp = ros::Time(0); // Start immediately
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
    }

    void open() {
        ROS_INFO("GripperInterface open");

        joint_state.position[0] = 0.0;
        joint_state.position[1] = 0.0;
        joint_state.position[2] = 0.0;
        joint_state.position[3] = 0.0;
        joint_state.position[4] = 0.0;
        joint_state.position[5] = 0.0;
        publish_jointstate();
    }

    void setPosition(double position) {
        joint_state.position[0] = position;
        joint_state.position[1] = position;
        joint_state.position[2] = position;
        joint_state.position[3] = position;
        joint_state.position[4] = position;
        joint_state.position[5] = position;
        publish_jointstate();
    }

#if 0
    boost::shared_ptr<urdf::Model> urdf;

    ////////////////////////////////////////////
    ///

    boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name) {
        boost::shared_ptr<urdf::Model> urdf(new urdf::Model);

        std::string urdf_str;
        // Check for robot_description in proper namespace
        if (nh.getParam(param_name, urdf_str)) {
            if (!urdf->initString(urdf_str)) {
                ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
                        nh.getNamespace() << ").");
                return boost::shared_ptr<urdf::Model>();
            }
        }            // Check for robot_description in root
        else if (!urdf->initParam("robot_description")) {
            ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
            return boost::shared_ptr<urdf::Model>();
        }
        return urdf;
    }

#endif

};
