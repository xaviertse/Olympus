#!/bin/bash

cd $PWD
source devel/setup.bash
roslaunch atg_ur10e_support ur10e_sim.launch
wait
