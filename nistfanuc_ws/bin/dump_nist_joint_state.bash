#!/bin/bash

p=/usr/local/michalos/nistfanuc_ws
cmd=( gnome-terminal )

cmd+=( --tab --title="master" --working-directory="$p" 
-e 'bash -c "source ./devel/setup.bash; rostopic echo nist_controller/robot/joint_states"')

# roslaunch robotduo checkersonly.launch
"${cmd[@]}"



