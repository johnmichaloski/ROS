#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
Test of http://wiki.ros.org/pykdl_utils

pykdl_utils contains kdl_parser.py, for parsing URDF objects from the robot_model_py stack into PyKDL trees and chains, and kdl_kinematics.py, for wrapping KDL kinematics calls, making kinematics requests in Python far simpler. jointspace_kdl_kin.py also contains a KDLKinematics superclass which subscribes to /joint_states, automatically filling the FK and jacobian requests with the current joint angles.
 
Github and ROS Dependencies: 
https://github.com/gt-ros-pkg/hrl-kdl/tree/hydro/pykdl_utils
https://github.com/ros/urdf_parser_py

'''

import rospy

from urdf_parser_py.urdf import URDF
#from urdf_parser_py.urdf import Robot
#from urdfdom_py.urdf import URDF

from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

# VERSION CHANGE - http://answers.ros.org/question/197609/how-to-read-a-urdf-in-python-in-indigo/
#robot = URDF.load_from_parameter_server(verbose=False)
robot = URDF.from_parameter_server()
base_link = robot.get_root()
end_link = "link_6"  #robot.link_map.keys()[len(robot.link_map)-1] # robot.links[6]
print end_link
tree = kdl_tree_from_urdf_model(robot)
print tree.getNrOfSegments()
chain = tree.getChain(base_link, end_link)
print chain.getNrOfJoints()

kdl_kin = KDLKinematics(robot, base_link, end_link)
q = kdl_kin.random_joint_angles()
print 'joints:', q
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
print 'pose:', pose

#q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
#if q_ik is not None:
#    pose_sol = kdl_kin.forward(q_ik) # should equal pose
#J = kdl_kin.jacobian(q)
#print 'q:', q
#print 'q_ik:', q_ik
#if q_ik is not None:
#    print 'pose_sol:', pose_sol
#print 'J:', J

