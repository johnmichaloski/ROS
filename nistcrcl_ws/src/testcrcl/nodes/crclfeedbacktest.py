

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_msgs.msg import KinematicSolverInfo, PositionIKRequest
from moveit_msgs.srv import GetKinematicSolverInfo, GetKinematicSolverInfoResponse, GetPositionFK, GetKinematicSolverInfo, GetPositionFKRequest, GetPositionFKResponse
from geometry_msgs.msg import Pose

from std_msgs.msg import Header

from nistcrcl.msg import CrclCommandMsg
from nistcrcl.msg import CrclStatusMsg
import time
import genpy
import roslib.message
import thread
import time
import subprocess


#from moveit_commander import RobotCommander, MoveGroupCommander


########################################
# This uses the Kinematic services which are now hidden in moveit
class Robot():
    def __init__(self):
        self.joint_state = JointState()
        self.linknames=[]

        rospy.wait_for_service('get_fk')
        #rospy.wait_for_service('testkin/GetKinematicSolverInfo')

        self.get_fk_proxy = rospy.ServiceProxy('get_fk', GetPositionFK, persistent=True)
        self.get_fk_solver_info_proxy = rospy.ServiceProxy('get_fk_solver_info', GetKinematicSolverInfo)

        solver_info = self.get_fk_solver_info_proxy()
        rospy.loginfo(solver_info)

        for name in solver_info.kinematic_solver_info.joint_names:
            self.joint_state.name.append(name)
            rospy.loginfo("Adding joint " + str(name))
            
        for name in solver_info.kinematic_solver_info.link_names:
            self.linknames.append(name)
            rospy.loginfo("Adding link " + str(name))
            
        #self.joint_state.position = [0] * len(self.joint_state.name)    
        for  name in self.joint_state.name:
            self.joint_state.position.append(0.0)
            
    def JointIndex(self, jointname):
        return  self.joint_state.name.index(jointname)
        
    def Update(self, jointvals)    :
        for i  in range(0,len(jointvals.name)):
            self.joint_state.position[self.JointIndex(jointvals.name[i])]=jointvals.position[i]
            
    def FK(self,jointvals):

        self.request = GetPositionFKRequest()
        self.request.fk_link_names = self.linknames

        self.request.robot_state.joint_state = self.joint_state # JointState()
        self.request.robot_state.joint_state.header.frame_id = 'base_link'
#        self.request.robot_state.joint_state.name = self.joints
#        self.request.robot_state.joint_state.position = [0] * len(self.joints)
#        self.request.robot_state.joint_state.position[0] = jointvals.position[0]
#        self.request.robot_state.joint_state.position[1] = jointvals.position[1]
#        self.request.robot_state.joint_state.position[2] = jointvals.position[2]
#        self.request.robot_state.joint_state.position[3] = jointvals.position[3]
#        self.request.robot_state.joint_state.position[4] = jointvals.position[4]
#        self.request.robot_state.joint_state.position[5] = jointvals.position[5]

        self.request.header.frame_id = "base_link"
        try:
            response = self.get_fk_proxy(self.request)
            #rospy.loginfo(response) 
            return response
        except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                self.FK(jointvals)

# This handles accepting a crcl command via subscribe to set goal joints and pose
class CrclCmd:
    def __init__(self):
        self.crclcommand=0
        self.joints=JointState()
        self.crclcommandnum=0
        self.pose=Pose()
        self.robot=Robot()
        
    def pplist(self,list):
        return ' '.join(['%5.3f'%x for x in list])

    def callback(self, data):
        self.joints=data.joints
        print["{0}={1:0.4f}".format(self.joints.name[i], self.joints.position[i]) for i  in range(0,len(self.joints.name))]
        self.crclcommand=data.crclcommand
        self.robot.Update(data.joints)
        response=self.robot.FK(self.joints.position)
        self.pose.position=response.pose_stamped[0].pose.position
        self.pose.orientation=response.pose_stamped[0].pose.orientation
        p= response.pose_stamped[0].pose.position 
        print "Service success!!! Pose Position=" + self.pplist([p.x,p.y,p.z])

 
########################################
 # This runs as a separate thread and simulates the update of the robot status
def updateStatusThread(pub, crcl):
    status = CrclStatusMsg()

    status.statuspose.position.x = 0.465
    status.statuspose.position.y = 0.0
    status.statuspose.position.z =  0.695
    status.statuspose.orientation.x =  0.0
    status.statuspose.orientation.y =  0.0
    status.statuspose.orientation.z =  0.0
    status.statuspose.orientation.w =  1.0
    status.eepercent=1.0
    while not rospy.is_shutdown():
        status.statusjoints=crcl.joints
        status.statuspose=crcl.pose
        status.crclcommandnum=crcl.crclcommandnum
        pub.publish(status)    
        time.sleep(1)

def runProcess(exe):    
    p = subprocess.Popen(exe, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    while(True):
      retcode = p.poll() #returns None while subprocess is running
      line = p.stdout.readline()
      yield line
      if(retcode is not None):
        break
    
if __name__ == '__main__':
    #time.sleep(10) # wait for ros core master to start

    print '============ Start ROS feedback of crcl topics...'

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('crclfeedback', anonymous=True,log_level=rospy.DEBUG)

    crcl=CrclCmd()
    for line in runProcess('rostopic list'.split()):
        print line
    
    rospy.Subscriber("crcl_command", CrclCommandMsg, crcl.callback)
    pub = rospy.Publisher('crcl_status', CrclStatusMsg, queue_size=10)
    updateStatusThread(pub, crcl)
    updateStatusThread(pub, crcl)
    #thread.start_new_thread(updateStatusThread(pub, crcl))
    
        # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

