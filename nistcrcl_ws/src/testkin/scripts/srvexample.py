#!/usr/bin/env python


# Import required Python code.
import rospy
import sys
from sensor_msgs.msg import JointState
from moveit_msgs.msg import KinematicSolverInfo, PositionIKRequest
from moveit_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoResponse, GetPositionFK, GetKinematicSolverInfo, GetPositionFKRequest, GetPositionFKResponse

import time

# http://www.programcreek.com/python/example/27316/rospy.ServiceProxy
# https://github.com/awesomebytes/my_pkg


# From http://forums.trossenrobotics.com/archive/index.php/t-4525.html
class get_fk():
    def __init__(self):
        rospy.init_node("fanuc_robot_get_fk")

        self.rate = 1
        r = rospy.Rate(self.rate)

        self.joints=[]
        self.links=[]

        rospy.wait_for_service('get_fk')
        #rospy.wait_for_service('testkin/GetKinematicSolverInfo')

        get_fk_proxy = rospy.ServiceProxy('get_fk', GetPositionFK, persistent=True)
        get_fk_solver_info_proxy = rospy.ServiceProxy('get_fk_solver_info', GetKinematicSolverInfo)

        solver_info = get_fk_solver_info_proxy()
        rospy.loginfo(solver_info)

        for joint in solver_info.kinematic_solver_info.joint_names:
            self.joints.append(joint)
            rospy.loginfo("Adding joint " + str(joint))
        for link in solver_info.kinematic_solver_info.link_names:
            self.links.append(link)
            rospy.loginfo("Adding link " + str(link))

        self.request = GetPositionFKRequest()
        self.request.fk_link_names = self.links

        self.request.robot_state.joint_state = JointState()
        self.request.robot_state.joint_state.header.frame_id = 'base_link'
        self.request.robot_state.joint_state.name = self.joints
        self.request.robot_state.joint_state.position = [0] * len(self.joints)
        self.request.robot_state.joint_state.position[0] = 0.0
        self.request.robot_state.joint_state.position[1] = 0.0
        self.request.robot_state.joint_state.position[2] = 0.0
        self.request.robot_state.joint_state.position[3] = 0.0
        self.request.robot_state.joint_state.position[4] = 0.0
        self.request.robot_state.joint_state.position[5] = 0.0

        self.request.header.frame_id = "base_link"

        while not rospy.is_shutdown():
            try:
                response = get_fk_proxy(self.request)
                print "Service success!!!"
                rospy.loginfo(response) 
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

            r.sleep()


if __name__ == '__main__':
    try:
        get_fk()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down python test forward kinematics service node...")

