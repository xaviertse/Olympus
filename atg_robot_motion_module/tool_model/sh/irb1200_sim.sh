#!/bin/bash

cd $PWD
source devel/setup.bash
roslaunch abb_irb1200_support irb1200_sim.launch
wait
