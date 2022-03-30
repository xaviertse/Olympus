#!/bin/bash

cd $PWD
source devel/setup.bash
roslaunch atg_ur5e_support ur5e_sim.launch
wait
