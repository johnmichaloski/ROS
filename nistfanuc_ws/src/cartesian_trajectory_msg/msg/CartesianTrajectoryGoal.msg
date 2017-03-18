#
# CartesianTrajectoryGoal
#

std_msgs/Header header  # A stamp of 0 means "execute now"
geometry_msgs/PoseStamped tool  # The frame which is being controlled
std_msgs/String[] posture_joint_names
CartesianTrajectoryPoint[] points
std_msgs/Duration time_from_start
geometry_msgs/Pose pose
geometry_msgs/Twist twist
std_msgs/Float64[] posture
#trajectory_msgs/JointTrajectory posture  # For determining the redundancy
  
CartesianTolerance path_tolerance  # Tolerance for aborting the path
CartesianTolerance goal_tolerance  # Tolerance for when reaching the goal is considered successful
std_msgs/Bool queue

