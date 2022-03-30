#!/bin/bash

sleep 2
cd $PWD
source devel/setup.bash
roslaunch abb_irb1200_support irb1200_setup.launch sim:=true robot_ip:=""
wait
