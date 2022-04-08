##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(atg_gui)
add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files


set(CMAKE_CXX_STANDARD 14)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/ros/kinetic;~/Qt/5.10.0/gcc_64")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)

#find_package(VTK 6.3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

find_package(VTK 8.1.1 REQUIRED)
# Fix a compilation bug under ubuntu 16.04 (Xenial)
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp roslib)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(Q~/diamond_ws_18-12-03/src_stCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/diamond_gui/*.hpp)

find_package(Qt5Widgets)
#find_package(Qt5Declarative)
find_package(Qt5 COMPONENTS Core Gui)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/atg_gui/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(atg_gui ${QT_SOURCES} ${project_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(atg_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS atg_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
qt5_use_modules(${PROJECT_NAME} Widgets)
