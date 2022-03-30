#!/bin/bash

cd $PWD
source devel/setup.bash
#roslaunch atg_abb_irb2400_support irb2400_interface.launch robot_ip:=192.168.2.4
roslaunch atg_abb_irb2400_support irb2400_sim.launch
wait
