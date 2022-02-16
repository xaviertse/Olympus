cmake_minimum_required(VERSION 2.8.3)
project(tool_model)
add_definitions(-std=c++11)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             roslib
             tf)

#find_package(PCL 1.8 REQUIRED) #josh added

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


#include_directories(${PCL_INCLUDE_DIRS})    #josh added
#add_definitions(${PCL_DEFINITIONS})         #josh added

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES masking
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(publish_coupon_node src/publish_coupon.cpp)       #josh added for tf
#target_link_libraries(publish_coupon_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})   #josh added for tf


