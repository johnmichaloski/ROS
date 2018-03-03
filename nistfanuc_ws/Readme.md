
# Readme for Real Time Crcl Trajectory Controller in ROS
 
----

Michaloski, John L. (Fed)

11/17/2016 5:13:00 PM

NistControllerReadme.docx



This document presents a Robot Operating System (ROS)  package for a trajectory motion and gripper open/close control that accepts Canonical Robot Control Language (CRCL) commands and reports robot status using ROS subscribe and advertise communication topics.

FYI THE UPGRADE TO ROS-KINETIC SEEMS TO HAVE BROKEN THE ROBOT KINEMATICS. TBD.

This implementation provides a simulation that is displayed in RVIZ yet differs from other ROS trajectory packages, e.g., moveit, in that it does not use the trajectory or kinematic functionality of moveit. It does use the Unified Robot Description Format (URDF). The URDF robot description is read using a C++ based on code developed by David Lu. Originally, kinematics were handled by the Kinematics and Dynamics Library (KDL) from Orocos to solve the forward and inverse kinematics of a robot represented in URDF.  Because KDL would not converge on a solution unless provided suitable hints and other issues in solving the kinematics, IKFast was used to solve the kinematics. In addition, the ikfast standalone solution for the Fanuc LR Mate 200 iD robot are available to perform forward and inverse kinematics.

The visualization of the Motoman robot below is to sort small and medium "gears" into appropriate gear holders is shown in an animated gif below (in this case on "vessels" which hold one type of gear, while a kit may hold varios size gears – or at least in this demo.)  In the code a vision system would generate JSON representation of the parts and instances of the parts. The demo code reads the JSON file using the boost Property Tree package. 

![Figure1](./images/image1.gif)


<p align="center">
**Figure 1 Animated gif of Motoman sorting gears**
</p>
The version information for the Real Time Crcl Trajectory Controller is:

 - ROS indigo 

 - OS - Ubuntu 12.04 (64-bit)

 - Package versions in Appendix I

Clearly improvements to the robot capabilities are in order. No collision is done so that the robot inverse kinematics chooses a solution where one of the joints has a negative Z, and would collide with the "floor".

 - Collision checking – first start with illegal inverse kinematic solutions

 - Separate destinations for robot versus object – robot wants to move to a little above slot and the gear wants to be in the slot at the bottom. Need to separate the destinations.

 - Mongo db for arhival and retrieval of objects

# Canonical Robot Control Language (CRCL) Background 

Canonical robot command language (CRCL) is part of the robot research at NIST. CRCL is a messaging language for controlling a robot. CRCL commands are executed by a low-level device robot controller. The usual source of CRCL commands is a plan/program execution system. CRCL is intended for use with devices typically described as industrial robots and for other automated positioning devices such as automated guided vehicles (AGVs). An AGV with a robotic arm attached may be regarded as a single robot responding to a single stream of CRCL commands or as two robots responding to two separate streams of CRCL commands.

Although CRCL is not a programming language, the commands are in the context of a session consisting of getting ready for activity, performing activities, and becoming quiescent. CRCL commands may be collected in files for testing purposes, but executing such files (by giving the commands in the order they occur in the file) is not be the normal operating mode of a robot. Because robots operate in uncertain and changing environment, the reliance on sensors to adjust for such disturbances makes canned scripts ineffective under real conditions. 

CRCL models a status message from a low-level robot controller. Status includes the position and orientation (Poses) that are the subject of CRCL commands. If any joint status reporting is done, it is assumed that the system sending canonical commands and the system executing them both know the kinematics of the robot and have the same numbering system for the joints, starting with 1. The two systems also have the same understanding of where the zero point is and which direction is positive for each joint. Status items for joints must be configured using a CRCl ConfigureJointReports command. For each joint for which anything is to be reported, ConfigureJointReports specifies:

 - whether joint position should be reported

 - whether joint torque or force should be reported

 - whether joint velocity should be reported

During a CRCL session, until a ConfigureJointReports command has been executed that sets the reporting status for a joint, default joint status is reported for that joint. The ConfigureJointReports command may be used more than once during a session to change joint status reporting.

# Software Architecture

The controller handles a Fanuc LRMate 200 iD and a robotiq two finger gripper, as shownin the figure below:

![Figure2](./images/image2.gif)


The robot and gripper are modeled in ROS URDF shown below:

![Figure4](./images/image4.gif)




The following diagram describes the ROS topic communication between modules in the contol system.

![Figure5](./images/image5.gif)


The Robot model combines the Fanuc LR Mate 200 id with the robotiq 2 finger gripper. The controller advertised updates to the /nist_controller/robot/joint_states which is read by the joint_state_publisher package. This communication is enabled in the launch file by the following snippet:

	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	    <param name="/use_gui" value="true"/>
	    <rosparam param="/source_list">[nist_controller/robot/joint_states]</rosparam>
	</node>

The architecture also supports CRCL command input and status reporting via a CRCL package. NIST CRCL package accepts CRCL commands and reports status to connected  ip and socket ports and then published  commands using the topic /crcl_command and subsribes to topic /crcl_status to receive feedback.  This CRCL communication is enabled in the launch file by the following snippet:

	<node name="nistcrcl" pkg="nistcrcl" type="nistcrcl" respawn="false" output="screen" >
	 <param name="crclip" value="127.0.0.1"/>
	 <param name="crclport" value="64444"/>
	</node>

	

This snippet loads the ROS node "nistcrcl" with the parameters specified as ip (i.e., crclip)  equal to "127.0.0.1" and the port (i.e., crclport) equal to 64444.



# Robot Kinematic Chain

The robot path is specified I terms of a "kinematic chain " made up of a series of homogeneous matrix transforms relation the manipulator to the task.

![Figure6](./images/image6.gif)


<p align="center">
**Figure 2 Position Equation**
</p>
This kinematic chain is evaluated many times a second, each time providing a new set of joint angles positions for the manipulator to follow. This kinematic chain will execute a function each sample period that returns a 4x4 Homogeneou Transform that defines the position and orientation of that element.  The Trajectory Generator will use the values in the kinematic chain to solve the kinematics for a robot. Sensor integration is accomplished with the same mechanism; transforms are determined in real time based on sensor input instead of by a static transform.

The MotionEquation class is responsible for assembling a kinematic chain. It uses a static formula to build a kinematics chain, and then each slot uses a callback to a boost statically bound function pointer. The default function pointer returns an identity function. 

A Kinematic Chain is assembled suing the make_equation method.  A chain is constructed providing a name, kinematic solver for the robot and then a a series of MotionEquation enums specify the equation layout .which form an equation with a left hand side, and a right hand side. The enumeration EQUALS divides the equation into the left and right hand sides. 

	  RCS::Pose Base(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
	  RCS::Pose Robot(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
	  RCS::Pose Gripper(Quaternion(0, 0, 0, 1), Vector3(.120, 0, 0));
	  RCS::Pose Table(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
	  RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.25, -.45, 0.35));
	  // Works
	  // RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, 0.695));
	  // RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.465, 0, 0.695));
	  //RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, .335));
	
	  KinematicChain::MotionEquation chain;
	  chain.make_equation("Test", kin,
	    KinematicChain::MotionEquation::BASE, 
	    KinematicChain::MotionEquation::ROBOT, 
	    KinematicChain::MotionEquation::TOOL, 
	    KinematicChain::MotionEquation::EQUALS,
	    KinematicChain::MotionEquation::TABLE, 
	    KinematicChain::MotionEquation::GOAL,
	    KinematicChain::MotionEquation::DONE
	    );
	  //chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
	  chain.SetPose( KinematicChain::MotionEquation::GOAL, GoalPose);
	  chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
	  chain.SetPose( KinematicChain::MotionEquation::TOOL, Gripper);
	  std::vector<double> joints = chain. Solve();



## Robot Gripper

In general, an end effector is the device at the end of a robotic arm, meant to interact with the environment (Wikipedia, 2016). The exact nature of this device depends on the application of the robot.  We are concerned with grasping objects and placing the objects somewhere else. This can be done with a vacuum gripper, but we are interested in the case of using grippers (with 2 fingers) to achieve object manipulation (grasping and releasing).

![Figure7](./images/image7.gif)


<p align="center">
**Figure 3 Fanuc LR Mate 200iD With Robotiq 2 Finger Gripper**
</p>
Robotiq's 2-Finger Adaptive Robot Gripper is modeled as the gripper since a ROS URDF description existed for its kinematics and a CAD model existed to described it visually. Of concern, is determining the gripper offset, that is what is the length offsets of the x,y,z axes of the gripper when it is attached to the robot.  To understand the xyz gripper offset, the URDF model describes link6 as having the xaxis point straight ahead, the yaxis points to the side and the z axis points up. Figure 4 shows the positioning of the Link 6 axis and the relationship to the gripper.

![Figure8](./images/image8.gif)


<p align="center">
**Figure 4 Fanuc Robot Arm Link 6 Axes**
</p>
So, in the URDF scenario, the  y axis is immaterial – the y axis determines the gripper opening, not offsets. While Figure 3 shows the offsets of concern the x and z axis. In our case the x axis describes the length of the gripper.  The robotiq   description can be found at http://robotiq.com/products/adaptive-robot-gripper/ gives tshe length of the gripper as 140 mm, which we use as the x translation offset.  Notices that the gripper up/down position changes, and this corresponds to a change in the z axis. Since this offset is the negative z direction (down), we will show later how -0.017 meters was determined to be the Z offset. 

A special CRCL command was added to allow a kinematic ring with gripper offset to be defined as something other than the identity matrix. Below, the RCS (real time control system) canonical command is given that describes the gripper offset pose as a constructor combination of an identity quaternion and translation offset.. 

	void AddGripperOffset(){
	    RCS::CanonCmd cmd;
	    cmd.crclcommandnum = crclcommandnum++;
	    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER_POSE;
	    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(
	            RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
	            tf::Vector3(0.140, 0.0, -0.017) )); // -0.01156)));
	    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
	}

When processed by the control system the gripper offset commands sets the kinematic pose component for the  gripper and its inverse. 

	else if (_newcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER_POSE) {
	                    gripperPose = Conversion::GeomMsgPose2RcsPose(_newcc.finalpose);
	                    invGripperPose = gripperPose.inverse();

Then any Cartesian motion that has a position and orientation to describe the motion changes the final point destination by postmultiplying the gripper inverse pose against the pose to determine the final robot pose.

	       RCS::Pose goalpose =  finalpose * Cnc.invGripperPose ;

 

Because we didn't actually have a robotiq 2-finger gripper, we couldn't just measure the z axis offset. Instead, a simple forward kinematic solution from the robotiq base joint to the final joint was calculated to give the Z axis. The hard coded solution will be shown to achieve the forward kinematic position, even though the links and axis of rotation and position and rotation  transform from the parent link can be determined from the URDF which was already parsed.

	RCS::Pose ComputeGripperOffset() {
	    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.0085 ,0 ,-.0041), Eigen::Vector3d(0, 0, 0)));
	    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(.04191, -.0306, 0), Eigen::Vector3d(1.5707, - 1.5707, 0)));
	    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, .00508, .03134), Eigen::Vector3d(3.1415, 0, 0)));
	    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(.04843 ,- .0127, 0), Eigen::Vector3d(-1.5707, - 1.5707, 0)));
	    AllM.push_back(ComputeUrdfTransform(0.0, Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0 ,.04196, - .0388), Eigen::Vector3d(0, 0, 0)));
	
	    RCS::Pose  pose = ComputeFk();
	    LOG_DEBUG << "Gripper Offset Pose " << RCS::DumpPoseSimple(pose).c_str();
	    }

This code manually loads the 5 robotiq URDF joint  information, and compute the Forward Kinematics (with zero joint angles) to determine the x and z offset. The x axis offset only goes to the final knuckle and not all the way down the gripper pinchers.



# RVIZ Visualization

The use of Rviz in simulation and visualization of the robot trajectory behavior is an important element in deploying the CRCL controlled robot. The CRCL includes Cartesian, joint and gripper control that is handled by the controller.

Rviz visualization is a nice robot visualization tool, but many of the elements are only explained in the context of implementations based on the Willow Garage PR2 robot. Thus, many of the tutorials although helpful, are bundled with other packages making it monolithic and often feel like coding with a heap of spaghetti. However, the source code is available and noodling around in the source code and by searching far and wide across the Internet, pearls of ROS programming can be found and where integrated into the package, and are hopefully understandable in this documentation. This section will attempt to explain how to use Rviz (without moveit planning and obstacle avoidance) to visualize a robot scene. True, eventually you will probably have to use moveit planning and obstacle avoidance, but one sip from a fire hose at a time.

The easies first step is to use roslaunch, in which you load a robot description and a "stripped down" version of Rviz.

	<param name="robot_description" command="$(find xacro)/xacro.py $(find fanuc_lrmate200id_support)/urdf/lrmate200id.xacro" />
	<node name="rviz" pkg="rviz" type="rviz">

When you do this, you will eventually see an RVIZ screen appear with the error condition of "Global Status". 

![Figure9](./images/image9.gif)




To rectify this error, click on the Fixed Frame text box (and possibly the base_link will appear in a combo box which you can select) or type in "base_link" or whatever is the base link in your URDF robot description. Below the error message disappears when base_link is entered.

![Figure10](./images/image10.gif)




Now, the problem is that no robot is visible. Obviously, not a good situation. To rectify this problem, click the [ADD} button above time in the lower left hand corner, and when the Create Visualization dialog box appears, select "Robot Model", as shown below:

![Figure11](./images/image11.gif)




Then, the robot that is described in the robot_description ROS parameter will appear in the RVIZ visualization, as shown below. The robot shown below is a Fanuc LR Mate 200 Id with a 2 finger robotiq gripper attached.





![Figure12](./images/image12.gif)




Next, moving the robot is important.  Two packages are useful in moving the robot: robot_state_publisher and  joint_state_publisher. 

 - robot_state_publisher allows you to publish the state of a robot to tf. Once the state gets published, it is available to all components in the system that also use tf. The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links, using a kinematic tree model of the robot.

 - joint_state_publisher publishes sensor_msgs/JointState messages for a robot. The package reads the robot_description parameter, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. joint_state_publisher  is used in conjunction with the robot_state_publisher node to also publish transforms for all joint states.

Of importance is the ROS parameter :ource_list", which is a list of topics that the "joint_state_publisher" node listens for sensor_msgs/JointState messages. Below, the "joint_state_publisher" node source list contains "nist_controller/robot/joint_states" topic which is listened to for new joint position to update the published joint_state. In this manner, the Real Time Crcl Trajectory Controller published either arm or gripper joints to the "nist_controller/robot/joint_states" topic, which the " joint_state_publisher" node listens to and republished on the "joint_states" topic that RVIZ is listening to for joint updates.

	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" output="screen" />
	
	<!-- We do not have a robot connected, so publish fake joint states -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	  <param name="/use_gui" value="true"/>
	  <rosparam param="/source_list">[nist_controller/robot/joint_states]</rosparam>
	</node>

****

# Manual Inserting of an STL File containing a scene object in RViz

A scene of objects for gripper manipulation by the Crcl Robot Controller in RVIZ must be built. Each object is imported into RVIZ as a Stereolithography Language (STL) file. To import an object, a 3D STL file is imported with the Scene Objects-Import File button. The STL format should be binary, not plain text. (?)

![Figure13](./images/image13.gif)


RVIZ is independent of moveit, so to get objects registered in the moveit Planning Scene, collision object programmatically to moveit (See http://wiki.ros.org/motion_planning_environment/Tutorials/Adding%20known%20objects%20to%20the%20collision%20environment) .



# RVIZ SCENE CREATION

This section covers the addition of objects to an RVIZ scene programmatically using C++. The scene creation uses the rviz-visual-tools ROS package, details found at  https://github.com/davetcoleman/rviz_visual_tools/blob/kinetic-devel/README.md.  This  Web site includes a tutorial on  rviz-visual-tools using RVIZ which  gives a good background to the ROS package  functionality.

For the Fanuc LR Mate 200iD, the addition of two objects will be illustrated to show how to use the rviz visual tools. Rviz visual tools uses the marker_array topic to publish object into rviz. Typically, one might use the moveit "collision objects"  to display scene objects in Rviz. 

Two objects will be added to the Rviz scene, a medium gear and a gear holder tray.  These scene objects were created in a CAD design system and have been produced by an 3D printing device. 3D printing devices use STL, so the STL files from these objects were imported and displayed Rviz using the displayMesh rviz-visual-tools method.

In addition, the method publishWall was developed that is based on rviz-visual-tools, to display a wall in Rviz.

![Figure14](./images/image14.gif)


<p align="center">
**Figure 5 Gear and Gear Holder Objects Displayed in Rviz Scene**
</p>
On the Ubuntu 12.4 platform that ROS was running indigo version , rviz-visual-tools was required to be installed. Using instructions from https://github.com/davetcoleman/rviz_visual_tools, the following command line performed the installation:

	sudo apt-get install ros-indigo-rviz-visual-tools

The initial test code was developed as C++ functions inside of a CPP implementation file.  The  rviz-visual-tools  package uses Eigen math library to specify positions and orientation, specifically, representing object locations as Homogeneous Transform matrices or Eigen::Affine3. Documentation for Eigen Affine representation can be found here: https://eigen.tuxfamily.org/dox-devel/group__TutorialGeometry.html





	#include <rviz_visual_tools/rviz_visual_tools.h>
	using namespace rviz_visual_tools;
	
	rviz_visual_tools::RvizVisualToolsPtr visual_tools;
	
	void InitSceneObject() {
	    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
	    visual_tools->deleteAllMarkers();
	    visual_tools->enableBatchPublishing();
	}
	
	void SetupSceneObject()
	{
		Eigen::Affine3d pose= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.5, 0, 0.0);
	 	Eigen::Affine3d pose4= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.25, -.45, 0.0);
	
	    bool  b;
	    if (!(b=visual_tools->publishMesh(pose, // or const geometry_msgs::Pose &pose
	            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
	            rviz_visual_tools::RED, // const colors &color = CLEAR,
	            0.035, // double scale = 1, 
	            "", // const std::string &ns = "mesh",
	            1))) { //  const std::size_t &id = 0))
	            std::cout << "SetupSceneObject() Failed\n";
	        }
	    visual_tools->triggerBatchPublish();
	#if 1
	    visual_tools->publishMesh(pose4, // or const geometry_msgs::Pose &pose
	            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
	            rviz_visual_tools::RED, // const colors &color = CLEAR,
	            0.035, // double scale = 1, 
	            "", // const std::string &ns = "mesh",
	            2);
	         visual_tools->triggerBatchPublish();
	#endif
	}



First, you need to modify ROS package.xml and CMakeList.txt to include references to rviz_visual_tools. The catkin build will fail if the rviz_visual_tools has not been installed. Without this package, include files, and the rviz_visual_tools library cannot be found.

Assuming the rviz_visual_tools  has been installed, then you need to include its header file to reference its functionality:

	#include <rviz_visual_tools/rviz_visual_tools.h>

Next, you need to declare a  RvizVisualToolsPtr from the namespace rviz_visual_tools to your code (either in your class or as a global variable):

	// For visualizing things in rviz
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

Please note, DO NOT instantiate the declaration with an instance, because it will attempt to do so before you have called ros_init in you main file, and will cause an error message. 

The function InitSceneObject was defined to instantiate visual_tools with a shared_ptr instance of RvizVisualTools, and to do some preliminary initialization (i.e., deleteAllMarkers and enableBatchPublishing) :

	void InitSceneObject() {
	    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
	    visual_tools->deleteAllMarkers();
	    visual_tools->enableBatchPublishing();
	}

Note, change the first parameter to the name of your robot's base frame (i.e., "base_link), and the second parameter to whatever name you'd like to use for the corresponding Rviz marker ROS topic (can see it rviz under marker array).

Now we create some scene objects using the function SetupSceneObject. In the following code we create two poses, one at xyz (0.5, 0, 0) with no orientation, and the other at pose (0.25, -.45, 0). Then the gear and the gear holder STL meshes are displayed by the publishMesh method. The color chosen was rviz_visual_tools::RED and each ID must be unique or only the last item with the id will be displayed. There is a namespace which was declared to be a "mesh", and a scaling factor (i.e., 0.35) when displaying the STL file. Unfortunately, other STL files were represented with millimeters, but by trial and error 0.35 seems to work.

	void SetupSceneObject()
	{
	  Eigen::Affine3d pose1= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.5,0,0);
	  Eigen::Affine3d pose2= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.25,-.45,0);
	
	  visual_tools->publishMesh(pose1,        "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
	   rviz_visual_tools::RED,
	   0.035, 
	   "mesh", // mesh namespace",
	   1); // id must be unique, 0 used package to assign id
	  visual_tools->triggerBatchPublish();
	
	  visual_tools->publishMesh(pose2,    "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
	   rviz_visual_tools::RED,
	   0.035, 
	   "", // no namespace",
	   2);
	  visual_tools->triggerBatchPublish();
	

Note, the use of an STL file mesh needs to be specified as a URI file (with leading file://) for Rviz to understand that it is a file and where the STL file can be located. Errors ini the ROS console will appear if you incorrectly specify the file to Rviz.

To test, use roslaunch to start the simple.launch file. This launch file will start rosmaster, rviz and set the robot_description ROS parameter. In Rviz, you need to make sure the Marker Array is established as an Rviz module.  To do this, press the 'Add' button at the bottom right and select Marker Array module.  Note, the marker array topic MUST be the same as the topic you specified in the RvizVisualTools constructor or Rviz will not listen to any marker arrays published by rviz_visual_tools.





## TRAJ Trajectory Planning Algorithms



Trajectory planning functions abbreviations:

 - CV means constant velocity, CA means constant acceleration,

 - CJ means constant jerk.



The Go Motion trajectory planning algorithms are based on smooth   velocity profiling with bounded speed, acceleration and jerk, called   "constant jerk" or "S-curve" velocity profiling. This gives smoother   control than "trapezoidal" velocity profiling, which transitions   instantaneously between acceleration and no acceleration and incurs   spikes in unbounded jerk. 



Constant-jerk (CJ) profiling is shown in Figure 1, a plot of the   speed versus time. There are 7 phases to the motion. Phase 1 is a   jerk phase, where the acceleration varies smoothly from 0 at time 0   to \a a1 at time \a t1 following the jerk (change in acceleration per unit   time) \a j0. Phase 2 is an acceleration phase, with   constant acceleration \a a1 throughout. Phase 3 is a jerk phase (or   de-jerk phase) with constant (negative) jerk slowing down the   acceleration from \a a1 to 0. Phase 4 is a constant speed phase at   speed \a v3. Phase 5 is a constant-jerk counterpart to phase 3,   where the deceleration varies smoothly from 0 to \a -a1. Phase 6 is   a constant-acceleration counterpart to phase 2. Phase 7 is a   constant-jerk counterpart to phase 1, where the deceleration varies   smoothly from \a -a1 to 0 and motion stops. 



![Figure15](./images/image15.gif)


<p align="center">
**Figure 6 Constant jerk velocity profiling.**
</p>


## Rviz display of the robot tf (transform) display

In order to get a visualization of the axes for each axis of you robot, rviz can offer this service is you ADD the "TF" module. Assuming you have started Rviz and configured it so that there is a robot description and the Robot Model module has been added to Rviz,  you can turn on the axes visualization for the links you desire to visualize them. Below, link_1 through link_6 have axis visualization enabled :



![Figure16](./images/image16.gif)






Here is another vantage point:

![Figure17](./images/image17.gif)


The X axis is indicated in red, the Y axis is indicated in green, and the Z axis is indicated in blue. (http://wiki.ros.org/rviz/DisplayTypes/TF).  Thus, in the scene above the bolt is located at (.25,-45,0) which is not the centroid of the object.

## Finding the pose of an object in the RVIZ scene

The STL meshes mapping into the Rviz scene use a pose to position the STL object. At this point in time, this Rviz mapping of the object pose is that it is not a centroid or it would be assumed the centroid of the pose to place the bolt object would give the bolt location – and it does not.

Instead trusty joint publisher GUI has sliders to move the joint values around to place the robot with the correct position and orientation to pick up the bolt. Suffice to say that it was not trivial centering the robot over the bolt but can be done. You can get the joint positions on the Joint State Publisher GUI :

![Figure18](./images/image18.gif)




Assuming you have started Rviz and configured it so that there is a robot description and the Robot Model module has been added to Rviz, you can read the position and orientation of link_6 which should place the robot in the correct position to grasp the bolt:

![Figure19](./images/image19.gif)


# Two Robot Cooperative Behavior

The addition of a Motoman with gripper was done with XACRO/URDF. Originally, there was a problem with attempting to move the base_link frame of the fanuc robot

	<link name="base_link">
	  <visual>
	    <origin rpy="0 0 0" xyz="0 -0.5 0"/>
	    <geometry>
	      <mesh filename="package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/base_link.stl" scale="1.0 1.0 1.0"/>
	    </geometry>
	    ....



But http://answers.ros.org/question/243910/easily-move-relative-base-position-in-urdf-for-2-seperate-robots/?answer=244198#post-id-244198 offered a concrete solution using a shared world frame that was used to create two robots each with a base offset sharing a common world frame. 



	<!-- File lrmate200id.xacro -->
	<?xml version="1.0"?>
	<robot name="fanuc_lrmate200id" xmlns:xacro="http://ros.org/wiki/xacro">
	  <xacro:include filename="$(find fanuc_lrmate200id_support)/urdf/lrmate200id_macro.xacro"/>
	   <xacro:include filename="$(find fanuc_lrmate200id_support)/urdf/robotiq_c2_model_macro.xacro"/>
	   <xacro:include filename="$(find motoman_sia20d_support)/urdf/sia20d_macro.xacro"/>
	    <xacro:fanuc_lrmate200id prefix="fanuc_"/>
	
	   <xacro:robotiq_c2_model prefix="fanuc_"/>
	    <xacro:motoman_sia20d prefix="motoman_"/>
	   <xacro:robotiq_c2_model prefix="motoman_"/>
	
	  <link name="world" />
	 
	  <!-- Dummy Link -->
	  <link name="link0" />
	  <joint name="world_joint" type="fixed">
	    <parent link="world" />
	    <child link="link0" />
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	    <axis xyz="0 0 1"/>
	  </joint>
	
	  <!-- First Robot -->
	  <joint name="fanuc_joint_0" type="fixed">
	    <parent link="link0" />
	    <child link="fanuc_base_link" />
	    <origin xyz="0 -0.5 0" rpy="0 0 0"/>
	  </joint>
	  
	  <joint name="fanuc_joint_6-tool0" type="fixed">
	    <!--      <origin xyz="0 0 0" rpy="${m_pi} ${-m_pi_2} 0" /> -->
	    <origin xyz="0 0 0" rpy="0 0 0" />
	    <parent link="fanuc_link_6" />
	    <child link="fanuc_robotiq_85_adapter_link" />
	    <axis xyz="0 0 0"/>
	  </joint> 
	  
	  <!-- Second Robot -->
	  <joint name="motoman_joint_0" type="fixed">
	    <parent link="link0" />
	    <child link="motoman_base_link" />
	    <origin xyz="0 0.5 0" rpy="0 0 0"/>
	  </joint>
	   <joint name="motoman_link_t-tool0" type="fixed" >
	     <!--  <origin xyz="0 0 0.0" rpy="0 0 -3.1416"/> -->
	     <origin xyz="0 0 0.0" rpy="0  -1.57  0"/>
	      <parent link="motoman_link_t" />
	      <child link="motoman_robotiq_85_adapter_link" />
	     </joint>
	  
	</robot>



An error oringally occurred, since the joint 'fanuc_joint_6-tool0' was not unique as it was defined in the file lrmate200id_macro.xacro and the file lrmate200id.xacro file. It was removed in the lrmate200id_macro.xacro file and was used to connect a robotiq gripper to it. Likewise this was done in the motoman macro xacro.

![Figure20](./images/image20.gif)


You musts set the base frame for tf to "world" for this to work.



But drawing the scene with bolts required that rviz_visual_tools use the world frame not the base_link frame that the original fanuc used. Otherwise the following error occurs:



![Figure21](./images/image21.gif)






****

# Gear Geometry

Therer are small, medium, and large gears.

![Figure22](./images/image22.gif)


Likewise, there are small, medium and large gear "vessels" that can hold the geats. Below are the dimensions for the small gear holder.

![Figure23](./images/image23.gif)


<p align="center">
**Figure 7 Small Gear Vessel Dimensions**
</p>

![Figure24](./images/image24.gif)


There is a little mixing of units here. The linear dimensions of the vessel are in inches. The linear units of ROS are always in meters. Thus, the offsets are described as values in meter units. While the vessel part is described in inch units. The part as described by 4 3/8 inches is equivalent to .11125 meters.

With much effort the STL mesh that describes this small gear holder was centered at its centroid (or middle of the part for both x and y). Then offsets were computed for each of the four "holders" of the gears. The vision system supplies the centroid of the gear vessel and from this the relative position of each gear holder can be computed. Included in the vision system is the rotation of the vessel, which is part of the computation to determine the gear holder positions.  The code to achieve this 

	slotpose = gearvesselpose * slotpose

 

where the pose is a transform matrix describing the position and orientation of the gear vessel and the slot position and orientation.  Initially, the slot pose orientation is the identity matrix. 

The  vision system provides a position (x,y) that we augment with a zero z position and a rotationn around z. Then, the pose of the gear vessel  is computed in the tf or transform namespace by the instance rotation provided as a Quaternion and the instance translational position from the origin in the world coordinate system.    

	    tf::Pose Shapes::GetInstancePose(Instance & instance) {
	        // rotation angle axis + position
	        return tf::Pose(GetInstanceRotation(instance),
	                tf::Vector3(instance.x, instance.y, instance.z));
	    }

The instance rotation is compute by providing the Quaternion constructor a vector around which to rotate (the z axis) and the amount of the rotation around z (i.e., instance.rotation).  The vision system provides the instance.rotation value.

	    tf::Quaternion Shapes::GetInstanceRotation(Instance & instance) {
	        return tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), instance.rotation);
	    }



# JSON Description of the World Model

In the described robot controller, JSON was used to describe the "world model" of the robot(s). As the name, implies the world model contains the model of the objects in the world. The world model of the robots consists of the dynamic objects that are manipulated by the robots (e.g., the gears) and static objects that are part of the scene (e.g., walls). 

JSON was used to describe the objects. JSON is an open-standard format that uses human-readable text to exchange hierarchical data represented in a tree consisting of attribute–value pairs. The JSON format is syntactically identical to the code for creating JavaScript objects as well as a subset of YAML. Further, there a "Non-SQL" data bases that use JSON as the schema representation and for defining data. 

The JSON structure is a hierarchical tree of nodes, branches, children, without cycles. Every JSON tree has exactly one root element. All other nodes are contained under this single root element. Nodes can be nested within another node to establish parent/child relationships. A branch is then described as a node and all the children under the node, and all the childrens children under that, ad infinitum.

	implicit root with children: "child1" and "child2"
	{ "child1": { "attr11": "value11" },
	  "child2": { "attr21": "value21" }
	}

"Parts" and "Instances" are the two primary JSON branches under the root. At this point in time, the boost Property Tree parses the JSON (as well as traditional ini format) for syntax validation and tree manipulcation API that was used to translate and interpret the JSON as a Shapes data repository with Part model containing Shape Geometry and other attributes, and instances which are instantiations of the part models. For example below is the JSON to describe the small and medium size gears and the vessels to hold the gears:

	{  
	   "parts":{  
	      "sku_small_gear":{  },
	      "sku_small_gear_vessel":{  },
	      "sku_medium_gear":{  },
	      "sku_medium_gear_vessel":{  }
	   },
	   "instances":{  
	      "sku_small_gear1":{  },
	      "sku_small_gear2":{  },
	      "sku_small_gear3":{  },
	      "sku_small_gear4":{  },
	      "sku_medium_gear1":{  },
	      "outline_sku_small_gear_vessel1":{  },
	      "sku_small_gear_vessel1":{  },
	      "sku_medium_gear_vessel1":{  }
	   }
	}



In the controller, the instances are drawn in ROS as RVIZ markers. The instance name given by the child branches under the instance branch (i.e., sku_small_gear1, sku_sm sku_small_gear_vessel1all_gear2,  sku_small_gear3,  sku_small_gear4, sku_medium_gear1, outline_sku_small_gear_vessel1, and sku_medium_gear_vessel1) is used as the name in the controller. Gear  "type" can be of any type of gear: small, medium or large. Gear trays can contain a combination of gear type holders. The distinction used herein, is that a gear vessel can hold all of the same gear types (e.g., sku_small_gear_vessel1 only can hold small gears), while a kit can contain a combination of gear types (e.g., sku_kit2  can hold small or medium or large gears). Each part describes the geometry and part of the containing slots for gears.

## Gear Tray

Below is a figure showing the STL visualization for a medium gear vessel/tray that will hold the medium gears.  The gear vessel design was specified in inches and had a few issues. First, the centroid of the vessel part was not in the middle of the XY plane. Second, the part's bottom in the Z axis  was negarive, so if it was placed on a flat surface, it would "in" the surface. Using FreeCAD, the part was centered in the middle, and the Z axis was raised so that all z values were greater than or equal to zero.

![Figure25](./images/image25.gif)


<p align="center">
**Figure 9 STL Visualization of the Medium Gear Vessel**
</p>
The medium gear vessel has 2 components: the part description and instances of the part.  The part description describes attributes that all instances of the part exhibit – part definition (including type, metatype, subtype, color) with specific types having further detail about its definition (e.g., a mesh has the location of the STL file describing the part for RVIZ.) A part "instance" uses it metatype to reference the part description, but adds items for position and rotation, which can be supplied by a vision system. There can be multiple "instances" of any "part". For example, 4 small gear instances can reference the small gear part design.

Walking through the JSON, the medium gear vessel JSON "part" description is shown below:

	"sku_medium_gear_vessel":{           "type":"primitive",         "metatype":"mesh",         "subtype":"solid",         "file":"file:///.../nistfanuc_ws/src/nist_robotsnc/worldmodel/medium_gear_holder_centeredZposge0.stl",         "scale":0.0254,         "color":"CYAN",         "centroid":"(4.5in, 4.5in), slot (2.25in, 2.25in)",         "contains":{ . . . }      }

Each part branch contains children: centroid, type, metatype, subtype, and color provide the information that describes the part to the controller and the simulation visualization. Specific types of parts (in this case mesh) have different additional child branches  to further describe the part profile. In the case of mesh, these include the "file"  which describes the location of the STL file and scale which describes the scaling factor to apply to the 

 - "centroid" child is used for documentation purposes, and is actually not used by the controller. 

 - "type" describes the part type, in this case a vessel, i.e., a holder of gears,

 - "metatype" describes the type of scene object, in this case,  an STL mesh, 

 - "subtype"  describes a refinement of the type, either solid or outline, of the equivalent graphical object. Some types cannot support all subtype, for example, a mesh cannot be an outline.

 - "color" is a child  branch of all the parts, and in this case provides the mesh color (i.e., "CYAN"), which will be translated from a string into an ROS RVIZ equivalent representation. Color is used to describe all instances, but can be overridden if an instance color branch is defined.

 - "contains" is a branch that describes the slots where gears can be inserted.   Each instance of this holder provides a state description (empty or full) of subbranches in its instance.  

A mesh part "type" contains these additional child attributes:

 - "file" is a child  branch specific to an metatype mesh, and provides a location of the STL mesh file, 

 - "scale" is a child  branch specific to an metatype mesh  that provides the scale factor to use for displaying the mesh, in this case the scale factor is the conversion from inch to meter, i.e., 0.0254.

Below the JSON for the part description child contains branch is expanded. The type of the part has a "contains" branch, which is a holder – it contains slots to hold gears. The "contains" branch has a child "metatype", which is a "name" in the "part" sub branch of the root that can fit into the contained slot. Included the "contains" description is the location on the part (an offset from the centroid of the part). The child JSON branch "position" describes a list which contains the x,y,z offset from the centroid. The child JSON branch "rotation" describes a rotation of the slot, if any. The child JSON branch "state" is a branch that must be in the instance definition. For an instance, the "state" is either "filled" or "empty". Thus, if it is "empty" it is able to hold a "free" gear. 

	"contains":{            "slot1":{                 "type":"holder",               "metatype":"sku_medium_gear",               "position":[  0.0396, 0.0396, 0.0 ],               "rotation":0,               "state":"tobedefined"            },. . .            
	           "slot4":{                 "type":"holder",               "metatype":"sku_medium_gear",               "position":[  -0.0396, -0.0396, 0.0 ],               "rotation":0,               "state":"tobedefined"            }
	    }     



Instances is a subbranch of the root JSON which contains the objects in the world model.  

	{  
	   "instances":{  
	      "sku_small_gear1":{  
	         "type":"gear",
	         "metatype":"sku_small_gear",
	         "color":"RED",
	         "position":[ 0.22989,-0.25126,  0.0 ],
	         "rotation":-0.82,
	         "state":"free"
	      }
	. . . 
	   }
	}

A simple instance  will be described which references a part model for its representation.

 - "type" is the category of the instance, in this case a gear. It could be a vessel or outline.

 - "metatype" gives the name in the part branch, or the part geometry model name. 

 - "color" is provides the object color (i.e., "RED"), which will be translated from a string into an ROS RVIZ equivalent representation. Color is used to describe all parts, but can be overridden if an instance color branch is defined.

 - "position" describes the x,y,z translation of the part in the world coordinate system. The list (i.e., numbers between the brackets "[]") is parsed into a tf::Vector3 representation. When combined with the "rotation" it describes a 4D representation that is then converted into a 6D posed, i.e., tf::Pose.  The position is typically provided by a vision system that can identify the position and rotation of the parts.

 - "rotation" describes rotation around the Z axis of the instance. Only the Z axis is required as the parts are all flat on the bottom and assumed to be resting flat on a surface. However, the part can be turned and the "rotation" attribute describes this amount. When combined with the "position" attribute it can be represented as a 6D pose (assuming zero rotation in X and Y axes.) The rotation is typically provided by a vision system that can identify the position and rotation of parts.

 - "state" describes the part or holder slot  stateness.  We assume initially that all parts are "free" when in fact the part could already be placed in a holder slot. This would only require simple mathematics to see if the part pose matches a holder container slot pose, however, the part may not be discernible when contained in a tray slot. Likewise, gear holder slots may be "filled" but will always be assumed to be "empty". It is TBD how this will be handled.

Of note, the position translation is described in the world coordinate system, but a transform is applied to bring the parts into a robot coordinate system, since there are two robots and the part instances are described from a Fanuc robot centered at (0,0,0) in  the world coordinate system. 

As another example, a small gear vessel JSON will be reviewed. Below is the JSON describing an instance (i.e., sku_small_gear_vessel1) of a small gear vessel in the world model. The type is a "holder" indicating that it is holder of gears. The metatype (i.e., " sku_small_gear_vessel" ) references a part model to indicate how the instance will be drawn. The subtype is solid indicating that it is not an outline. The position is derived from a vision system and in combination with the "rotation" provides the pose of the gear vessel (position and orientation). The "contains" child describes the state for each holder slot 1..4. As stated, initially the states of all holder slots is empty as it is not clear how an initial  placement of a gear in a holder could be detected. 

			"sku_small_gear_vessel1" :
			{
				"type" : "holder",
				"metatype": "sku_small_gear_vessel",
				"subtype" : "solid",
				"position":[ 0.3155,0.13662, 0.0],
				"rotation": -1.4,
				"contains":
				{ "slot1": { "state" : "empty"} ,
				"slot2": { "state" : "empty"} ,
				"slot3": { "state" : "empty"} ,
				"slot4": { "state" : "empty"} 
				}
			},

Once a gear holder slot has been filled, the "state" changes to "filled" to prevent another gear from being placed into the slot.



# Gear Placement Algorithm

/





# Robot Configuration

A serial manipulator is the most common industrial robot and can be simply described as an arm with a end effector. If you want the robot to manipulate something the end effector is a gripper.  Robots arms can be made with 4, 5, 6, 7 or more axes. Serial robots usually have six joints, because it requires at least six degrees of freedom to place a manipulated object in an arbitrary position and orientation in the workspace of the robot. The combination of position and orientation is referred to as a pose, typically 6 degree of freedom.   The position and orientation of a robot's end effector are derived from the joint positions by means of a kinematic model of the robot arm. The forward kinematics (FK) is concerned with the mapping from joint positions to end-effector pose. The inverse kinematics (IK) is concerned with the mapping from an end-effector pose into to end-effector pose.

These robots are designed as a series of links connected by motor-actuated joints that extend from a base to an end-effector (or gripper). These robot axes can change position and are called joints. Joints can generally be revolute (or rotary) or prismatic. Often the robots take on anthropomorphic characteristics with the joints modeled after the human body with a base (akin to the waist) and a shoulder, elbow and wrist. An articulated wrist has multiple degrees of freedom at the wrist – typically called roll, pitch and yaw.  Serial manipulators are the most common industrial robots.

Much like a human can pick up something with their elbow pointing up or down so can a robot. The redundancy of solutions to grasp something leads to the need to describe a configuration of robot joints to achieve a goal pose. For example, often there may be an obstacle (such as a table) that you do want to collide with, so it is desired to command the robot configuration to have the elbow up not down where it could collide with the table.

A robot arm can generally reach a pose (position and orientation) in a number of configurations.  The configuration of the robot can change in order to reach a desired position and orientation. For example, an "elbow" could be up or down, the base joint forward or swiveled around 180o or even an articulated wrist  can have the wrist swiveled 180o to reach an object. The number of joint affects the number of solutions. The number of axes determines the number of solutions, and as is often the case with 7 or more degree of freedom robots, it is required to fix the solution of one joint or an infinite number of robot poses. 

This section will attempt to categorize the various configurations that different styles of manipulator robots may encounter and provide a coding mechanism in which to describe the configuration.  Generally, for non-redundant robots, each configuration can be mapped into revolute ranges for each joint. For example, a shoulder up could result in a min and max joint angle of: 

To understand the requirement of arm configuration, let's start with an example. For example, the Fanuc LR Mate 200id is a 6-axis robot arm with an articulated wrist is shown below. One anomaly occurs when the Fanuc LR Mate can reach the same pose by offsetting a twist in joint 4 with a twist in joint 6 leading to an infinite number of joint solutions to attain a pose. This is known as a singularity. A singularity is defined as a robot configuration in which the joint positions no longer completely define the robot pose.

![Figure26](./images/image26.gif)


**Figure 10 Fanuc LR Mate  Home Position - Joints all zero**

Instead, we will concentrate on the configuration of the Fanuc LR Mate base, shoulder, elbow and wrist joints. The base joint can be pointed straight ahead or swiveled around by 180 degrees, in which case the shoulder would compensate by bending backward. 

![Figure27](./images/image27.gif)


**Figure 11Fanuc LR Mate  Base rotated and Shoulder Flipped**











![Figure28](./images/image28.gif)



![Figure29](./images/image29.gif)



![Figure30](./images/image30.gif)



![Figure31](./images/image31.gif)



![Figure32](./images/image32.gif)




![Figure33](./images/image33.gif)



![Figure34](./images/image34.gif)




![Figure36](./images/image36.gif)



![Figure38](./images/image38.gif)






## Joint Range Exercise

The algorithm chooses from min to max range for all joints in a robot. The joint values act like an odometer or other sequential distance measurement instrument – joint values are incremented and each digit causes a rollover in the next digit when the maximum is reached, and that value is reset to the minumum. Using this "odometer" like joint increment, each set of joint values are feed into a Forward Kinematics computation to compute the Cartesian pose. Then the Inverse Kinematics is performed on the pose and if successful, the exact same joint values should be computed. 



![Figure39](./images/image39.gif)



![Word2Markdown](./images/word2markdown.jpg)

[Word2Markdown](https://github.com/johnmichaloski/SoftwareGadgets/tree/master/Word2Markdown)
