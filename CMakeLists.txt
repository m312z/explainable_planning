cmake_minimum_required(VERSION 2.8.11)
project(explainable_planning)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  actionlib
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
  explainable_planning_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

########
## QT ##
########

# Installed 5.10.1 from https://www.qt.io as Ubuntu 14.04 has 5.2 as default
# set(Qt5Core_DIR "/home/michael/Qt/5.10.1/gcc_64/lib/cmake/Qt5Core")
# set(Qt5_DIR "/home/michael/Qt/5.10.1/gcc_64/lib/cmake/Qt5")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport Charts)

add_definitions( -std=c++11 -fPIC)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs diagnostic_msgs rosplan_planning_system explainable_planning_msgs
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  include
)

## do some qt things
qt5_wrap_cpp(main_window_SRC include/explainable_planning/mainwindow.h)

## Declare cpp executables
add_executable(rpexplainer src/RPExplainer.cpp)
add_executable(xaipinterface src/ExplanationInterface.cpp)
add_executable(main_window src/mainwindow.cpp ${main_window_SRC})

add_dependencies(rpexplainer ${catkin_EXPORTED_TARGETS})
add_dependencies(xaipinterface ${catkin_EXPORTED_TARGETS})
add_dependencies(main_window ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rpexplainer ${catkin_LIBRARIES})
target_link_libraries(xaipinterface ${catkin_LIBRARIES})
target_link_libraries(main_window ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets Qt5::Charts)

#############
## Install ##
#############

#install(TARGETS rpmovebase
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
