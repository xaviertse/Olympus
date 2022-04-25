#!/bin/bash

sleep 2
cd $PWD
source devel/setup.bash
#roslaunch atg_ur10_support ur10_setup.launch robot_setup_config_name:="ur10e"
#roslaunch atg_ur10_support ur10_e_setup.launch
roslaunch atg_ur10e_support ur10e_setup.launch
wait
