cmake_minimum_required(VERSION 2.8.3)
project(dual_people_leg_tracker)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# OpenCV
find_package( OpenCV REQUIRED )


# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

include_directories(.)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  sensor_msgs
  laser_geometry
  tf
  visualization_msgs
  people_msgs
  people_tracking_filter
  image_geometry
  dynamic_reconfigure
  rosbag
  message_generation
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

generate_messages(
   DEPENDENCIES
   std_msgs
   rosgraph_msgs
   tf2_msgs
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS people_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs message_runtime)