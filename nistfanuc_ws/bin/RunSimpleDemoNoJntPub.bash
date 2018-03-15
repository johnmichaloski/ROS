#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

p=/usr/local/michalos/nistfanuc_ws
cmd=( gnome-terminal )

cmd+=( --tab --title="roslaunch" --working-directory="$p" 
-e 'bash -c "source ./devel/setup.bash; exec bash"')

cmd+=( --tab --title="master" --working-directory="$p" 
-e 'bash -c "source ./devel/setup.bash; roslaunch robotduo simplenojntpub.launch"')

"${cmd[@]}"
