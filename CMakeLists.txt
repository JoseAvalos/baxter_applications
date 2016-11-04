cmake_minimum_required(VERSION 2.8.3)
project(baxter_cpp)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg visualization_msgs kinect)

###########################################################################

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Kine-solver-rbdl and RBDL packages
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(OSIK_CONTROL REQUIRED osik-control)
PKG_CHECK_MODULES(RBDL REQUIRED rbdl)
IF (RBDL_FOUND)
  SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
ENDIF (RBDL_FOUND)

# Find QPOASES
FIND_PACKAGE(qpOASES)

## Find catkin macros and libraries
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  sensor_msgs
  kinect
  roscpp
  rospack
  roslib
  tf
  visualization_msgs
  geometry_msgs
  tf2_msgs
)

CATKIN_PACKAGE()

###########
## Build ##
###########

INCLUDE_DIRECTORIES(
  include
  ${RBDL_INCLUDE_DIRS}
  ${OSIK_CONTROL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${OSIK_CONTROL_LIBRARY_DIRS}
)

ADD_LIBRARY(baxter_cpp SHARED
  src/markers.cpp
  src/tools.cpp
  )


#########################################################################




## Declare ROS messages and services
#add_message_files(FILES kinectbody.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Build talker and listener
#include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(writing src/writing.cpp)
target_link_libraries(writing ${catkin_LIBRARIES})
## add_dependencies(writing beginner_tutorials_generate_messages_cpp)

add_executable(OSIK_baxter envio_baxter.cpp)
TARGET_LINK_LIBRARIES( OSIK_baxter
  baxter_cpp
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARIES}
  ${RBDL_URDFReader_LIBRARIES}
  ${OSIK_CONTROL_LIBRARIES}
  ${qpOASES_LIBRARIES}
)


add_executable(xtion_OSIK_baxter baxter_xtion.cpp)
TARGET_LINK_LIBRARIES( xtion_OSIK_baxter
  baxter_cpp
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARIES}
  ${RBDL_URDFReader_LIBRARIES}
  ${OSIK_CONTROL_LIBRARIES}
  ${qpOASES_LIBRARIES}
)

add_executable(Data_Kinect filtro_kinect.cpp)
TARGET_LINK_LIBRARIES( Data_Kinect
  ${catkin_LIBRARIES}
)
#add_executable(Publisher publisher.cpp)
#TARGET_LINK_LIBRARIES( Publisher
##  baxter_cpp
#  ${catkin_LIBRARIES}
#)
