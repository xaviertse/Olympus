#!/bin/bash

#sleep 2
#cd /home/diamond/diamond_ws/
#cd /home/joshc/diamond_ws_18-12-03/
cd $PWD
source devel/setup.bash
#rosrun masking code_upload _scriptname:=$1
roslaunch atg_robot_motion send_urscript.launch robotIP_:=$1 robotPort_:=$2 scriptname_:=$3
#wait
