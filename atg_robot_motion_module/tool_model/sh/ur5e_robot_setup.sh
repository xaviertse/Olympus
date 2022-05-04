#!/bin/bash

sleep 2
cd $PWD
source devel/setup.bash
roslaunch atg_ur5e_support ur5e_setup.launch
wait
