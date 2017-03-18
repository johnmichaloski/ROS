#
# CartesianTrajectoryResult
#
std_msgs/Int32 error_code  # 0 if successful
CartesianTrajectoryPoint cartesian_state
trajectory_msgs/JointTrajectoryPoint joint_state
geometry_msgs/Twist pose_error
geometry_msgs/Twist twist_error
CartesianTolerance path_tolerance  # Current tolerance used for the path
