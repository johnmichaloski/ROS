#!/bin/bash

p=/usr/local/michalos/nistfanuc_ws
cmd=( gnome-terminal )

c = rostopic pub -1 nist_controller/robot/joint_states sensor_msgs/JointState '{header: auto, name: ['fanuc_joint_1'], position: [ 0.5418], velocity: [], effort: []}'

cmd+=( --tab --title="fake" --working-directory="$p" 
-e 'bash -c "source ./devel/setup.bash; ${c}"')

# roslaunch robotduo checkersonly.launch
"${cmd[@]}"
#rostopic pub -1 nist_controller/robot/joint_states sensor_msgs/JointState '{header: auto, name: ['fanuc_joint_1'], position: [ 0.5418], velocity: [], effort: []}'

