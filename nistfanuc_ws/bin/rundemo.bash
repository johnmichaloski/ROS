#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

p=/usr/local/michalos/nistfanuc_ws
cmd=( gnome-terminal )
cmd+=( --tab --title="checkers demo"  --working-directory="$p" 
-e "bash -e $p/bin/startrviz.bash")
"${cmd[@]}"
