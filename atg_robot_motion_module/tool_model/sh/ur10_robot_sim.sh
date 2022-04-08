#!/bin/bash

cd $PWD
source devel/setup.bash
roslaunch atg_ur10_support ur10_sim.launch
wait
