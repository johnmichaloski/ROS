
This is ROS respository to demonstrate simple grasping and ROS capabilities. A Fanuc LR Mate 200 iD has
a robotiq 2 finger gripper modeled in URDF in ROS and displayed in RVIZ. A simple bang-bang controller (no
smooth trajectory motion) is used to move the checkers. Often the checker motions have a sloppy feel
but it cuts down on the code size. Typically ROS moveit is used, but there are too many "moving parts"
to easily integrate pick/place robotics.

So far,  besides using the core ROS tools, the following packages or code have been very useful:

- David Lu arm_kinematics
- Dave Coleman rviz_visual_tools
- Dave Hershberger, David Gossow, Josh Faust RVIZ

Here is an animated gif of what is supposed to happen:


![Figure1](./images/checkers.gif?raw=true)
