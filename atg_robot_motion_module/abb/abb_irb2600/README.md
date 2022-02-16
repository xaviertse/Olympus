# ABB
The [abb_experimental][] repository contains additional packages.


## Contents

[original] abb_irb2600_support is copied from abb_experiental package 
[custom] atg_abb_irb2600_support is custom with dependency on abb_irb2600_suppot
[custom] atg_abb_irb2600_moveit_config is created using urdf/xacro in atg_abb_irb2600_support
[custom] irb2600_12_165_ikfast_manipulator is created using ikfast tutorial but modified for ros descartes compatibility
[custom] abb_irb2600_12_165_descartes defines the robot model used for ros descartes dependent on irb2600_12_165_ikfast_manipulator

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`kinetic`).


## Naming Convention

All robot support packages and MoveIt configurations follow the naming conventions as described in [REP-I0007][].




[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/abb
[abb_experimental]: https://github.com/ros-industrial/abb_experimental
[ikfast_tutorial]: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html

