How Kinematics Functions Work
============
  The kinematics functions relate the joints to the "kinematic control
  point" KCP. The forward kinematics calculate the KCP from the joint
  values, and the inverse kinematics calculate the joint values from
  the KCP. 

  Often one would like to add another transform from the KCP to an
  application "end control point" ECP. This is commonly called the
  "tool transform," with the KCP called the "wrist frame" and the ECP
  called the "tool frame". As tools are placed on the manipulator,
  the tool transform changes.

  <small><sup>0</sup><sub>K</sub></small>T is the KCP. This is the
  frame of the kinematics functions, where the 0 means the world
  coordinate system.

  <small><sup>0</sup><sub>E</sub></small>T is the ECP. This is the
  frame of motion control, program coordinates, position limits, home
  position and the display.

  <small><sup>K</sup><sub>E</sub></small>T is the tool transform.
  The tool transform is specified as the position and orientation
  of the tool's end control point, ECP, with respect to the kinematic
  control point, KCP,

  <small><sup>K</sup><sub>E</sub></small>T = | <small><sup>K</sup><sub>E</sub></small>R <sup><small>K</small></sup>P<sub><small>Eorg</small></sub> |

  i.e., the position and orientation of the tool tip expressed with
  respect to the kinematic control point (aka wrist frame).

  To go from the ECP to the KCP, we postmultiply the ECP by the
  inverse tool transform to get the KCP:

  <small><sup>0</sup><sub>K</sub></small>T = <small><sup>0</sup><sub>E</sub></small>T * <small><sup>E</sup><sub>K</sub></small>T

  that is, 
  
  ECP * inverse tool transform = KCP
  
  or in code as go_pose_pose_mult(&ECP, &tool_transform_inv, &KCP).

  Results from the forward kinematics functions are in the KCP, and
  must be transformed into the ECP when sent out as status. To go from the KCP to the ECP, we postmultiply the KCP by the tool transform to get the ECP:

  <small><sup>0</sup><sub>E</sub></small>T = <small><sup>0</sup><sub>K</sub></small>T * <small><sup>K</sup><sub>E</sub></small>T

  that is,

  KCP * tool transform = ECP

  or in code as go_pose_pose_mult(&KCP, &tool_transform, &ECP).

  Changing the tool transform is tricky. Even the motion queue is
  empty and the controller is holding the ECP constant, changing the
  tool transform will cause a jump in the KCP and a corresponding jump
  in the joint values coming out of the inverse kinematics. To handle
  this, we need to change the ECP when changing the tool transform:

  <small><sup>0</sup><sub>K</sub></small>T * <small><sup>K</sup><sub>Enew</sub></small>T = <small><sup>0</sup><sub>Enew</sub></small>T, KCP * new tool transform = new ECP

  When the tool is changed, you would see a jump in the displayed
  position, but no jump in actual position.

  To avoid inconsistencies between points on the motion queue in
  various ECPs, the tool transform can only be changed when the
  motion queue is empty. The configuration state table for changing
  the tool transform ensures this, then updates the queue position
  with the new ECP.

  The ECP is in the traj status buffer as 'cmd_position'.

  The KCP is in the traj status buffer as 'kcp'.

  The tool transform and inverse are in the traj settings buffer
  as 'tool_transform' and 'tool_transform_inv'.

  To convert from a pose A in the ECP to the pose in the KCP, premultiply
  by the tool transform:

  <sup><small>K</small></sup>A = <small><sup>K</sup><sub>E</sub></small>T * <sup><small>E</small></sup>A


Homing a Joint
======
  The servo controllers work entirely in their initial frame. When   they are homed, the position at which they homed is stored in a   latch variable. The trajectory controller can then change to the homed frame, and adjust with an offset to maintain initial frame
  values for servo.

Go Motion Stop
----
1. don't need to stop an empty or stopping queue 
2. drop all pending motions from the queue
3.  point our attention at the current motion 
4.  we want to stop now, the current queue time
5.   'endtime' will the the revised stop time
6.   Call go_traj_cj_stop for each joint or tran/rot motion, so that each stops as fast as it can. The longest to stop will set the revised stop time, and each will be extended to stop at that longest time.  Note that the specptr values are incremental for the move, as the original distance passed to `go_traj_cj_compute` was the incremental distance.
7. now 'endtime' is the longest stop time, so extend all the moves to that time, and revise the joint end positions and the queue end position 'there'
8. revise the queue's end position
9. now 'endtime' is the longest stop time, so extend both tran and rot to that time
10. now the tran and rot are extended; now we need to recompute   the new end translation and rotation.

