
#Readme for Real Time Crcl Trajectory Controller in ROS 
----

Michaloski, John L. (Fed)
7/21/2016 4:32:00 PM
NistControllerReadme.docx

This document presents the robot controller with trajectory motion and gripper open/close control developed for the Robot Operating System (ROS) package that accepts Canonical Robot Control Language (CRCL) commands and reports robot status using ROS subscribe and advertise communication topics.
This implementation provides a simulation that is displayed in RVIZ yet differs from other ROS trajectory packages, e.g., moveit, in that it does not use the trajectory or kinematic functionality of moveit. It does use the Unified Robot Description Format (URDF) and Kinematics and Dynamics Library (KDL) from orocos to soved the forward and inverse kinematics of a robot represented in URDF.
The version information for the Real Time Crcl Trajectory Controller is:
 - ROS indigo 
 - OS - Ubuntu 12.04 (64-bit)
 - Package versions in Appendix I
#Canonical Robot Control Language (CRCL) Background 
Canonical robot command language (CRCL) is part of the robot research at NIST. CRCL is a messaging language for controlling a robot. CRCL commands are executed by a low-level device robot controller. The usual source of CRCL commands is a plan/program execution system. CRCL is intended for use with devices typically described as industrial robots and for other automated positioning devices such as automated guided vehicles (AGVs). An AGV with a robotic arm attached may be regarded as a single robot responding to a single stream of CRCL commands or as two robots responding to two separate streams of CRCL commands.
Although CRCL is not a programming language, the commands are in the context of a session consisting of getting ready for activity, performing activities, and becoming quiescent. CRCL commands may be collected in files for testing purposes, but executing such files (by giving the commands in the order they occur in the file) is not be the normal operating mode of a robot. Because robots operate in uncertain and changing environment, the reliance on sensors to adjust for such disturbances makes canned scripts ineffective under real conditions. 
CRCL models a status message from a low-level robot controller. Status includes the position and orientation (Poses) that are the subject of CRCL commands. If any joint status reporting is done, it is assumed that the system sending canonical commands and the system executing them both know the kinematics of the robot and have the same numbering system for the joints, starting with 1. The two systems also have the same understanding of where the zero point is and which direction is positive for each joint. Status items for joints must be configured using a CRCl ConfigureJointReports command. For each joint for which anything is to be reported, ConfigureJointReports specifies:
 - whether joint position should be reported
 - whether joint torque or force should be reported
 - whether joint velocity should be reported
During a CRCL session, until a ConfigureJointReports command has been executed that sets the reporting status for a joint, default joint status is reported for that joint. The ConfigureJointReports command may be used more than once during a session to change joint status reporting.
#Robot Kinematic Chain
The robot path is specified I terms of a "position equation" made up of a series of homogeneous matrix transforms relation the manipulator to the task.
<CENTER>
![Figure1](./images/image1.jpg?raw=true)
</CENTER>

<p align="center">
**Figure 1 Position Equation**
</p>
This position equation is evaluated many times a second, each time providing a new set of joint angles positions for the manipulator to follow. This type of transform will execute a function each sample period containing an equation to define its values.  The Trajectory Generator will use the new values in the position equation. Sensor integration is accomplished in the same manner; new transforms are determined by sensor input instead of by equations.
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

#RVIZ Visualization
The use of Rviz in simulation and visualization of the robot trajectory behavior is an important element in deploying the CRCL controlled robot. The CRCL includes Cartesian, joint and gripper control that is handled by the controller.
Rviz visualization is a nice robot visualization tool, but many of the elements are only explained in the context of implementations based on the Willow Garage PR2 robot. Thus, many of the tutorials although helpful, are bundled with other packages making it monolithic and often feel like coding with a heap of spaghetti . However, the source code is available and noodling around in the source code and search far and wide across the internet, pearls of ROS programming can be found. This section will attempt to explain how to use Rviz (without moveit planning and obstacle avoidance ) to visualize a robot scene. True, eventually you will probably have to use moveit planning and obstacle avoidance, but one sip from a fire hose at a time.
The easies first step is to use roslaunch, in which you load a robot description and a "stripped down" version of Rviz.

	<param name="robot_description" command="$(find xacro)/xacro.py $(find fanuc_lrmate200id_support)/urdf/lrmate200id.xacro" />
	<node name="rviz" pkg="rviz" type="rviz">
When you do this, you will eventually see an RVIZ screen appear with the error condition of "Global Status". 
<CENTER>
![Figure2](./images/image2.jpg?raw=true)
</CENTER>


To rectify this error, click on the Fixed Frame text box (and possibly the base_link will appear in a combo box which you can select) or type in "base_link" or whatever is the base link in your URDF robot description. Below the error message disappears when base_link is entered.
<CENTER>
![Figure3](./images/image3.jpg?raw=true)
</CENTER>


Now, the problem is that no robot is visible. Obviously, not a good situation. To rectify this problem, click the [ADD} button above time in the lower left hand corner, and when the Create Visualization dialog box appears, select "Robot Model", as shown below:
<CENTER>
![Figure4](./images/image4.jpg?raw=true)
</CENTER>


The, the robot that is described in the robot_description ROS parameter will appear in the RVIZ visualization, as shown below. The robot shown below is a Fanuc LR Mate 200 Id with a 2 finger robotiq gripper attached.


<CENTER>
![Figure5](./images/image5.jpg?raw=true)
</CENTER>


Next, moving the robot is important.  Two packages are useful in moving the robot: robot_state_publisher and  joint_state_publisher. 
 - robot_state_publisher allows you to publish the state of a robot to tf. Once the state gets published, it is available to all components in the system that also use tf. The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links, using a kinematic tree model of the robot.
 - joint_state_publisher publishes sensor_msgs/JointState messages for a robot. The package reads the robot_description parameter, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. joint_state_publisher  is used in conjunction with the robot_state_publisher node to also publish transforms for all joint states.
Of importance is the ros parameter source_list", which is a list of topics that the "joint_state_publisher" node listens for sensor_msgs/JointState messages. Below, the "joint_state_publisher" node source list contains "nist_controller/robot/joint_states" topic which is listened to for new joint position to update the published joint_state. In this manner, the Real Time Crcl Trajectory Controller published either arm or gripper joints to the "nist_controller/robot/joint_states" topic, which the " joint_state_publisher" node listens to and republished on the "joint_states" topic that RVIZ is listening to for joint updates.



	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" output="screen" />
	
	<!-- We do not have a robot connected, so publish fake joint states -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	  <param name="/use_gui" value="true"/>
	  <rosparam param="/source_list">[nist_controller/robot/joint_states]</rosparam>
	</node>

<p align="center">
**0**
</p>
#Manual Inserting of an STL File containing a scene object in RViz
A scene of objects for gripper manipulation by the Crcl Robot Controller in RVIZ must be built. Each object is imported into RVIZ as a Stereolithography Language (STL) file. To import an object, a 3D STL file is imported with the Scene Objects-Import File button. The STL format should be binary, not plain text. (?)
<CENTER>
![Figure6](./images/image6.jpg?raw=true)
</CENTER>

RVIZ is independent of moveit, so to get objects registered in the moveit Planning Scene, collision object programmatically to moveit (See http://wiki.ros.org/motion_planning_environment/Tutorials/Adding%20known%20objects%20to%20the%20collision%20environment) .

#RVIZ SCENE CREATION
This section covers the addition of objects to an RVIZ scene programmatically using C++. The scene creation uses the rviz-visual-tools ROS package, details found at  https://github.com/davetcoleman/rviz_visual_tools/blob/kinetic-devel/README.md.  This  Web site includes a tutorial on  rviz-visual-tools using RVIZ which  gives a good background to the ROS package  functionality.
For the Fanuc LR Mate 200iD, the addition of two objects will be illustrated to show how to use the rviz visual tools. Rviz visual tools uses the marker_array topic to publish object into rviz. Typically, one might use the moveit "collision objects"  to display scene objects in Rviz. 
Two objects will be added to the Rviz scene, a medium gear and a gear holder tray.  These scene objects were created in a CAD design system and have been produced by an 3D printing device. 3D printing devices use STL, so the STL files from these objects were imported and displayed Rviz.
<CENTER>
![Figure7](./images/image7.jpg?raw=true)
</CENTER>

<p align="center">
**Figure 3 Gear and Gear Holder Objects Displayed in Rviz Scene**
</p>
On the Ubuntu 12.4 platform that ROS was running , rviz-visual-tools was required to be installed. Using instructions from https://github.com/davetcoleman/rviz_visual_tools, the following command line performed the installation:

	sudo apt-get install ros-kinetic-rviz-visual-tools
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
Not DO NOT instantiate the declaration with an instance, because it will attempt to do so before you have called ros_init in you main file, and will cause an error message. 
The function InitSceneObject was defined to instantiate visual_tools with a shared_ptr instance of RvizVisualTools, and to do some preliminary initialization (i.e., deleteAllMarkers and enableBatchPublishing) :

	void InitSceneObject() {
	    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
	    visual_tools->deleteAllMarkers();
	    visual_tools->enableBatchPublishing();
	}
Note, change the first parameter to the name of your robot's base frame (i.e., "base_link), and the second parameter to whatever name you'd like to use for the corresponding Rviz marker ROS topic (can see it rviz under marker array).
Now we create some scene objects using the function SetupSceneObject. In the following code we create two poses, one at xyz (0.5, 0, 0) with no orientation, and the other at pose (0.25, -.45, 0). Then the gear and the gear holder STL meshes are displayed by the publishMesh method. The color chosen was rviz_visual_tools::RED and each ID must be unique or only the last item with the id will be displayed. There is a namespace which was left blank, and a scaling factor (i.e., 0.35) when displaying the STL file. Unfortunately, other STL files were represented with millimeters, but by trial and error 0.35 seems to work.

	void SetupSceneObject()
	{
	  Eigen::Affine3d pose1= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.5,0,0);
	  Eigen::Affine3d pose2= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.25,-.45,0);
	
	  visual_tools->publishMesh(pose1,        "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
	   rviz_visual_tools::RED,
	   0.035, 
	   "", // no namespace",
	   1); // id
	  visual_tools->triggerBatchPublish();
	
	  visual_tools->publishMesh(pose2,    "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
	   rviz_visual_tools::RED,
	   0.035, 
	   "", // no namespace",
	   2);
	  visual_tools->triggerBatchPublish();
	
Note, the use of an STL file mesh needs to be specified as a URI file (with leading file://) for Rviz to understand that it is a file and where the STL file can be located. Errors ini the ROS console will appear if you incorrectly specify the file to Rviz.
To test, use roslaunch to start the simple.launch file. This launch file will start rosmaster, rviz and set the robot_description ROS parameter. In Rviz, you need to make sure the Marker Array is established as an Rviz module.  To do this, press the 'Add' button at the bottom right and select Marker Array module.  Note, the marker array topic MUST be the same as the topic you specified in the RvizVisualTools constructor or Rviz will not listen to any marker arrays published by rviz_visual_tools.
##Available Colors
This package helps you quickly choose colors - feel free to send PRs with more colors as needed

	BLACK,
	BLUE,
	BROWN,
	CYAN,
	DARK_GREY,
	GREEN,
	GREY,
	LIME_GREEN,
	MAGENTA,
	ORANGE,
	PINK,
	PURPLE,
	RED,
	WHITE,
	YELLOW,
	TRANSLUCENT_LIGHT,
	TRANSLUCENT,
	TRANSLUCENT_DARK,
	RAND,
	CLEAR,
	DEFAULT // i.e. 'do not change default color'


##TRAJ Trajectory Planning Algorithms

Trajectory planning functions abbreviations:
 - CV means constant velocity, CA means constant acceleration,
 - CJ means constant jerk.

The Go Motion trajectory planning algorithms are based on smooth   velocity profiling with bounded speed, acceleration and jerk, called   "constant jerk" or "S-curve" velocity profiling. This gives smoother   control than "trapezoidal" velocity profiling, which transitions   instantaneously between acceleration and no acceleration and incurs   spikes in unbounded jerk. 

Constant-jerk (CJ) profiling is shown in Figure 1, a plot of the   speed versus time. There are 7 phases to the motion. Phase 1 is a   jerk phase, where the acceleration varies smoothly from 0 at time 0   to \a a1 at time \a t1 following the jerk (change in acceleration per unit   time) \a j0. Phase 2 is an acceleration phase, with   constant acceleration \a a1 throughout. Phase 3 is a jerk phase (or   de-jerk phase) with constant (negative) jerk slowing down the   acceleration from \a a1 to 0. Phase 4 is a constant speed phase at   speed \a v3. Phase 5 is a constant-jerk counterpart to phase 3,   where the deceleration varies smoothly from 0 to \a -a1. Phase 6 is   a constant-acceleration counterpart to phase 2. Phase 7 is a   constant-jerk counterpart to phase 1, where the deceleration varies   smoothly from \a -a1 to 0 and motion stops. 

<CENTER>
![Figure8](./images/image8.jpg?raw=true)
</CENTER>

<p align="center">
**Figure 2 Constant jerk velocity profiling.**
</p>




**** 
Autogenerated from Microsoft Word by [Word2Markdown](https://github.com/johnmichaloski/SoftwareGadgets/tree/master/Word2Markdown)