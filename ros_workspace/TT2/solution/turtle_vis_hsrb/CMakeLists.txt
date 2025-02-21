cmake_minimum_required(VERSION 2.8.3)
project(turtle_vis_hsrb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

##############################################################################
# Qt Environment
##############################################################################

#find_package(Qt4 REQUIRED)
#SET(QT_USE_QTOPENGL TRUE)
#SET(QT_USE_QTNETWORK TRUE)

#include(${QT_USE_FILE})
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


##############################################################################
# Messages, Services and Actions
##############################################################################
add_message_files(
   DIRECTORY msg
   FILES
   DesiredPose.msg
)

## Generate services 
add_service_files(
  DIRECTORY srv
  FILES
  send_desired_pose.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${turtle_vis_hsrb}
  CATKIN_DEPENDS roscpp rospy std_msgs tf tf_conversions visualization_msgs message_run
#  DEPENDS system_lib 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} #Needed to find all headers generated by the uic (Qt)
)

# Define the library target
add_library(turtle_ros_tutorial_hsrb src/myClass/TurtleClass.cpp)
target_link_libraries(turtle_ros_tutorial_hsrb ${roscpp_LIBRARIES} ${catkin_LIBRARIES} )

set_target_properties(turtle_ros_tutorial_hsrb
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_dependencies(turtle_ros_tutorial_hsrb turtle_vis_hsrb_generate_messages_cpp)
add_dependencies(turtle_ros_tutorial_hsrb turtle_vis_hsrb_gencpp)

##############################################################################
# Sections Library Enable for Releases
##############################################################################
link_directories(${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Sections Tutorials
##############################################################################

add_executable(turtle_vis_hsrb_node src/turtle_vis_node.cpp)
add_dependencies(turtle_vis_hsrb_node turtle_vis_hsrb_generate_messages_cpp)
target_link_libraries(turtle_vis_hsrb_node ${catkin_LIBRARIES} ${roscpp_LIBRARIES} turtle_ros_tutorial_hsrb)

add_executable(turtle_control_hsrb_node src/turtle_control_node.cpp)
add_dependencies(turtle_control_hsrb_node turtle_vis_hsrb_generate_messages_cpp)
target_link_libraries(turtle_control_hsrb_node ${catkin_LIBRARIES} ${roscpp_LIBRARIES} turtle_ros_tutorial_hsrb)

add_executable(turtle_set_position_hsrb_node src/turtle_set_position.cpp)
add_dependencies(turtle_set_position_hsrb_node turtle_vis_hsrb_generate_messages_cpp)
target_link_libraries(turtle_set_position_hsrb_node ${catkin_LIBRARIES} ${roscpp_LIBRARIES} turtle_ros_tutorial_hsrb)

