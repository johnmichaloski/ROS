#NIST ROS Control of Robotiq 2 Finger Gripper

6/22/2016 10:29:17 AM 


This document describes the software hurdles in developing  a simple gripper client/server. The URDF and visualization are based on the ROS consortium packages for handling 2 and 3 finger Robotiq grippers.


##Notation

- **ROS**	Robot Operating System
- **URDF**	Unified Robot Description Format

##Installation
**1) Set up gripper_ws ROS workspace**

Clone gripper_ws package from https://github.com/johnmichaloski


**2) Set up ROS and ROS Industrial Robotiq Package**

We assume a ROS and a ROS gripper workspace has been set up.

Shaun Edwards of SWRI has setup up a robotiq gripper repository  that incorporates the major elements required of this demo - a  URDF and STL meshes to visualize the gripper.

Clone the robotiq repository from https://github.com/ros-industrial into the gripper_ws src directory.

The package within robotiq that is used is the 2 finger visualization (i.e., robotiq_c2_model_visualization).

Unfortunately, if you build the entire robotiq repository, it requires the modbus fix:

**3) Build**

Hopefully the package.xml and the CMakefile.txt files are correctly configured for use with ROS. I use catkin build (not catkin_make). 

	> catkin build

**4) Run**

	> source devel/setup.bash
	> roslaunch demogripperactionclient rviz.launch

##Hacks

ROS nodes that are run in rviz.launch are:

	joint_state_publisher
	robot_state_publisher
	rviz
	nist_gripper_action_server
	nist_gripper_action_client

Of note, since this is a simulation you **MUST** have `joint state publisher`. One obvious reason it that `nist_gripper_action_server` and `joint state publisher`  both publish to the same topic (i.e.,`joint_states`) and you would have a racing condition if both are enabled. So you would think you could only run `nist_gripper_action_server` and not `joint state publisher` but rviz posts a RobotModel error, which was out of my league to figure out. Instead, I made `nist_gripper_action_server` publish to a source on the "source_list" that 
`joint state publisher` was monitoring, and this worked. Within the launch file, you add the rosparam  `source_list` and add the ROS topic that `nist_gripper_action_server` will publish to. Below is launch xml code to do this:

	 <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
	    <rosparam param="source_list">["nist_controller/robot/joint_states"]</rosparam>
	</node>

Inside the Gripper server "hardware" interface, the `"nist_controller/robot/joint_states` topic is advertised, as shown below:

	joint_pub= _nh.advertise<sensor_msgs::JointState>("nist_controller/robot/joint_states", 10);

and then the gripper joint state is published to this topic, which is read by the `joint state publisher` and republished as `joint_states` for rviz to subscribe.

    sensor_msgs::JointState joint_state;
	...
	joint_pub.publish(joint_state);
 

The simulation uses the Robotiq visualization, which means the URDF file.  This is set as a parameter in the launch file, so the gripper is visualized in rviz.
 
	  <param name="robot_description" command="cat $(find robotiq_c2_model_visualization)/urdf/robotiq_c2_model.urdf" />

The `use_gui` param was set for `joint state publisher` so sliders were available to open and close the robotiq gripper. Then, the `rostopic` command was used to see what the robotiq gripper considered open and closed setting. `rostopic` is a command-line tool for displaying debug information about ROS Topics, including publishers, subscribers, publishing rate, and ROS Messages. Thus, the command
	
	> rostopic echo joint_states 

provided the values sent to robotiq gripper, which were published by the NIST gripper server "hardware" interface when open or close were called.
