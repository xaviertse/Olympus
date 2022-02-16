#!/bin/bash

sleep 2
cd $PWD
source devel/setup.bash
roslaunch atg_abb_irb2400_support irb2400_setup.launch sim:=true robot_ip:="192.168.2.4"
wait
