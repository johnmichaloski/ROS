

import sys
import copy
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nistcrcl.msg import CrclCommandMsg
import time


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.crclcommand)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("crcl_command", CrclCommandMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	print '============ Starting crcl testing'
	time.sleep(10) # wait for ros core master to start
	print '============ Listening crcl...'
	listener()





