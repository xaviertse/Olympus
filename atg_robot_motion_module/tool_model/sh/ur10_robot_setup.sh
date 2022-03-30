#!/bin/bash

sleep 2
cd $PWD
source devel/setup.bash
roslaunch atg_ur10_support ur10_setup.launch
wait
