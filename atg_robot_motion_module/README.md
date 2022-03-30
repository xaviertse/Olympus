# Installation
Follow atg_gui_module

# Module contains robot motion module and individual robot arm and tool model libraries

# How to use:
## Initialisation and visuzlisation of robot setup:
##### Run shell script written for launching selected robot configurations
```
<tool_model package>/sh/ur10_robot_setup.sh
<tool_model package>/sh/ur10e_robot_setup.sh
<tool_model package>/sh/irb2400_robot_setup.sh
<tool_model package>/sh/irb2600_robot_setup.sh
<tool_model package>/sh/irb1200_robot_setup.sh
```

Shell script -> roslaunch \<robot support package\> \<robot_setup\>.launch \<options\>

Robot luanch will read these static files:
- \<robot support package\>/urdf/\<robot_workspace\>.xacro
- \<tool_model package\>/urdf/\<tool\>.xacro

and these variable yaml files: (which is rewritten with params in ATG_GUI when executed)
- \<tool_model package\>/config/env_params.yaml
- \<tool_model package\>/config/obj_params.yaml
- \<tool_model package\>/config/tcp_params.yaml

Luanch file initialises ROS, robot parameters and RViz for visualisation, laucnhed node subscribes to robot messages and waits for inputs (from robot motion)



## Launch Robot Motion Generation:
##### Run shell script written for launching selected robot configurations
```
<tool_model package>/sh/ur10_robot_sim.sh
<tool_model package>/sh/ur10e_robot_sim.sh
<tool_model package>/sh/irb2400_sim.sh
<tool_model package>/sh/irb2600_sim.sh
<tool_model package>/sh/irb1200_sim.sh
```

Shell script -> roslaunch \<robot support package\> \<robot_sim\>.launch

Launch file intialises robot state and environment state and runs atg_robot_motion node

Input reads from static location <input file> for waypoints, I/O states and various KPV parameters

Output UR toolpath into static location /home/data/cache/urscript/URScript.txt

Output ABB toolpath into static location /home/data/cache/RAPID/abb_script.mod

Note that output files will be replaced every time a toolpath operation triggers, stored toolpath is suppose to be temp for use within instance



## Send robot motion script to robot:
##### Run shell script written for launching selected robot configurations
```
<atg_robot_motion package>/sh/urscript_send.sh
<atg_robot_motion package>/sh/abb_rapid_send.sh
```

##### urscript_send.sh runs the following:
roslaunch atg_robot_motion send_urscript.launch robotIP_:=$1 robotPort_:=$2 scriptname_:=$3
##### abb_rapid_send.sh runs the following:
wput -v -u $FILE ftp://"$USER":$PSWD@$HOST/atg/
