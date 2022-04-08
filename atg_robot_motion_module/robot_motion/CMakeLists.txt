cmake_minimum_required(VERSION 2.8.3)
project(atg_robot_motion)
add_definitions(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_planner
  descartes_moveit
  descartes_utilities
  moveit_msgs
  moveit_ros_planning_interface 
  atg_ur10_descartes
  atg_ur5e_descartes
  atg_ur10e_descartes
  abb_irb2400_descartes
  abb_irb2600_12_165_descartes
  abb_irb1200_7_70_descartes
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES masking
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_motion.cpp
  src/init_ros.cpp
  src/init_descartes.cpp
  src/move_home.cpp
  src/generate_trajectory.cpp
  src/plan_path.cpp
  src/run_path.cpp
  src/load_parameters.cpp
  src/generate_urscript.cpp 
  src/generate_rapid_script.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(atg_robot_motion_node src/robot_motion_main.cpp)
target_link_libraries(atg_robot_motion_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(urscript_upload src/send_script.cpp)
target_link_libraries(urscript_upload ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(read_robot_state src/read_robot_state.cpp)
target_link_libraries(read_robot_state ${PROJECT_NAME} ${catkin_LIBRARIES})

