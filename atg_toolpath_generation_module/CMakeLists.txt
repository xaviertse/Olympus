##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(atg_toolpath_generation)
add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################
# init_qt: Let's do the CMake job for us
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/ros/kinetic;~/Qt/5.10.0/gcc_64")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(PCL 1.8 REQUIRED)
find_package(VTK 8.1.1 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
catkin_package()

add_library(${PROJECT_NAME}
    src/atg_toolpath_generation.cpp
    )


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(atg_toolpath_contour src/atg_toolpath_contour.cpp)
target_link_libraries(atg_toolpath_contour ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(atg_toolpath_zigzag src/atg_toolpath_zigzag.cpp)
target_link_libraries(atg_toolpath_zigzag ${PROJECT_NAME} ${catkin_LIBRARIES})
<<<<<<< HEAD
=======

add_executable(atg_toolpath_point src/atg_toolpath_point.cpp)
target_link_libraries(atg_toolpath_point ${PROJECT_NAME} ${catkin_LIBRARIES})
>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
