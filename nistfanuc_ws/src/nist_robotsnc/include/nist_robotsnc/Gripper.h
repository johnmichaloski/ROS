

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
//#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include "nist_robotsnc/RCS.h"
#include "nist_robotsnc/Debug.h"
#include "nist_robotsnc/Globals.h"

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
protected:
    ros::Publisher joint_pub;
    std::vector<std::string> joint_names;
    const double degree = M_PI / 180;
    sensor_msgs::JointState joint_state;
    std::string prefix;
public:

    GripperInterface() {
    }

    virtual void init(ros::NodeHandle &nh, std::string prefix, bool bPublish = false) {
        ROS_INFO("GripperHwInterface init");
        try {
            this->prefix = prefix;
            //_nh.getParam("controller_joint_names", joint_names);
            if (joint_names.size() == 0) {
                joint_state.name.resize(6);
                joint_state.position.resize(6);
                joint_state.name[0] = prefix + "robotiq_85_left_knuckle_joint"; // 0 to 90o
                joint_state.name[1] = prefix + "robotiq_85_right_knuckle_joint"; // 0 to 90o
                joint_state.name[2] = prefix + "robotiq_85_left_inner_knuckle_joint"; // 0 to 90o
                joint_state.name[3] = prefix + "robotiq_85_right_inner_knuckle_joint"; // 0 to 90o
                joint_state.name[4] = prefix + "robotiq_85_left_finger_tip_joint"; // 0 to 90o
                joint_state.name[5] = prefix + "robotiq_85_right_finger_tip_joint"; // 0 to 90o
            } else {

                joint_state.position.resize(joint_names.size());
            }
            if (bPublish)
                joint_pub = nh.advertise<sensor_msgs::JointState>(Globals.joint_state_topic, 10);
        } catch (...) {
            ROS_ERROR("GripperInterface Failed to init");

        }
    }

    std::vector<std::string> JointNames() {
        return joint_state.name;
    }

    sensor_msgs::JointState closeSetup() {
        ROS_INFO("GripperInterface close");
        joint_state.header.stamp = ros::Time(0); // Start immediately
        joint_state.position[0] = 0.500;
        joint_state.position[1] = 0.500;
        joint_state.position[2] = 0.500;
        joint_state.position[3] = 0.500;
        joint_state.position[4] = -0.500;
        joint_state.position[5] = -0.500;
        return joint_state;
    }

    void close() {
        ROS_INFO("GripperInterface close");
        publish_jointstate();
    }

    void publish_jointstate() {
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
        joint_pub.publish(joint_state);
    }

    sensor_msgs::JointState openSetup() {
        ROS_INFO("GripperInterface open");
        joint_state.header.stamp = ros::Time(0); // Start immediately

        joint_state.position[0] = 0.0;
        joint_state.position[1] = 0.0;
        joint_state.position[2] = 0.0;
        joint_state.position[3] = 0.0;
        joint_state.position[4] = 0.0;
        joint_state.position[5] = 0.0;
        return joint_state;
    }

    void open() {
        ROS_INFO("GripperInterface open");
        publish_jointstate();
    }

    sensor_msgs::JointState setPosition(double position) {
        joint_state.header.stamp = ros::Time(0); // Start immediately
        joint_state.position[0] = position;
        joint_state.position[1] = position;
        joint_state.position[2] = position;
        joint_state.position[3] = position;
        joint_state.position[4] = -position;
        joint_state.position[5] = -position;
        return joint_state;
    }
};
