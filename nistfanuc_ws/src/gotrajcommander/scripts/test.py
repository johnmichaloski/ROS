


import inspect
import rospy
import tf
from geometry_msgs.msg import Pose, Quaternion, Vector3,PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from gotrajcommander  import GoTraj, GoTrajParams

rospy.init_node('test_gotraj')
 
gm = GoTraj()
print "GoTraj()"

print gm.IsDone()

joints = JointState()
joints.header = Header()
joints.header.stamp = rospy.Time.now()
joints.name = ['fanuc_joint_1', 'fanuc_joint_2', 'fanuc_joint_3', 'fanuc_joint_4','fanuc_joint_5', 'fanuc_joint_6' ]
joints.position = [0,0,0,0,0,0]
joints.velocity = []
joints.effort = []

tojoints = JointState()
tojoints.header = Header()
tojoints.header.stamp = rospy.Time.now()
tojoints.name = ['fanuc_joint_1', 'fanuc_joint_2', 'fanuc_joint_3', 'fanuc_joint_4','fanuc_joint_5', 'fanuc_joint_6' ]
tojoints.position = [0,1,2,3,2,1]
tojoints.velocity = []
tojoints.effort = []

jtparams=[GoTrajParams(1.,10.,100.),GoTrajParams(1.,10.,100.),GoTrajParams(1.,10.,100.),
GoTrajParams(1.,10.,100.),GoTrajParams(1.,10.,100.),GoTrajParams(1.,10.,100.)]

t=0.1
#InitWrap(joints, t)
gm.Init(joints, t)
print "gm.Init()"
gm.InitJoints(joints, tojoints, jtparams,True)
while ( not gm.IsDone() ):
	jts=gm.NextJoints()
	#print "Next joints", ', '.join(map(str,  jts.position))

home=Pose()

home.position.x =0
home.position.y =0
home.position.z =0
home.orientation = Quaternion(0,0,0,1)
there=Pose()
there.position.x =0
there.position.y =1
there.position.z =2
there.orientation = Quaternion(0,0,0,1)

# This creates its own "Pose" close to geometry messages but not identical, 
# Python seems to map them into each other ok?
gm.InitPose(home,there, GoTrajParams(1.,10.,100.),GoTrajParams(1.,10.,100.))

#bflag=True
#while(bflag):
	#pose=gm.NextPose()
	#whatup = inspect.getmembers(pose)
	#print whatup
	#bflag=False
i=0
while ( not gm.IsDone() ):
	p=PoseStamped()
	pose=gm.NextPose()
	p.pose=pose
	#print "Next Pose=", str(pose.position.x),":", str(pose.position.y),":" ,str(pose.position.z),":"
	print str(i)," Geometry Pose=", str(p.pose.position.x),":", str(p.pose.position.y),":" ,str(p.pose.position.z),":"
	i+=1
	if(i>8): gm.InitStop()

