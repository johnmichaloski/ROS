

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from nistcrcl.msg import CrclCommandMsg
from nistcrcl.msg import CrclStatusMsg
import time
import genpy
import roslib.message
import thread
import time

#	for i in range(len(joints.name)):
#		name = joints.name[i]
#		print name


def PrintJoints(joints):
	print["{0}={1:0.4f}".format(joints.name[i], joints.position[i]) for i  in range(0,len(joints.name))]

class CrclCmd:
	def __init__(self):
		self.crclcommand=0
		self.joints=JointState()
		self.crclcommandenum=0

	def callback(self, data):
		self.joints=data.joints
		#for i in range(len(data.joints.name)):
		#	name = data.joints.name[i]
		#print data.joints.name[0]  # CRCL index - THERE IS NO NAME
		#print data.joints.position[0]
		#print genpy.message.strify_message(data) # This is really great debug info
		PrintJoints(data.joints)
		self.crclcommand=data.crclcommand

	def feedbacktalker(self, pos):
		global pub
		stat = CrclStatusMsg()
		stat.statusjoints=self.joints
		stat.crclcommandnum=self.crclcommandnum
		rospy.loginfo(rospy.get_caller_id() + "I sent %s", stat.crclcommandenum) 
		pub.publish(stat)

#while not rospy.is_shutdown():


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.crclcommand)

def updateStatusThread(sub, crcl)
	status = CrclStatusMsg()
	status.statuspose.position.x = 0.465
	status.statuspose.position.y = 0.0
	status.statuspose.position.z =  0.695
	status.statuspose.orientation.x =  0.0
	status.statuspose.orientation.y =  0.0
	status.statuspose.orientation.z =  0.0
	status.statuspose.orientation.w =  1.0
	while 1 :
		status.statusjoints=crcl.joints
		status.crclcommandnum=crcl.crclcommandnum
		pub.publish(status)	
		time.sleep(1)

if __name__ == '__main__':
	time.sleep(10) # wait for ros core master to start

	print '============ Start crcl...'

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
        rospy.init_node('crclfeedback', anonymous=True)
	moveit_commander.roscpp_initialize(sys.argv)
	crcl=CrclCmd()

	pub = rospy.Publisher("crcl_status', CrclStatusMsg, queue_size=10)
	rospy.Subscriber("crcl_command", CrclCommandMsg, crcl.callback)

	thread.start_new_thread(updateStatusThread(pub, crcl))
	
    	# spin() simply keeps python from exiting until this node is stopped
   	rospy.spin()

