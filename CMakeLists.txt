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

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)


include_directories(.)
include_directories(${Eigen_INCLUDE_DIRS})

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
  image_geometry
  dynamic_reconfigure
  rosbag
  message_generation
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(dual_people_leg_tracker
            src/models/occlusion_model.cpp
            src/models/advanced_measmodel_pos.cpp
            src/models/advanced_sysmodel_pos_vel.cpp
            src/distributions/multivariate_gaussian_pos_vel.cpp
            )

catkin_package(INCLUDE_DIRS 
                  include # The exported include paths (i.e. cflags) for the package
               LIBRARIES dual_people_leg_tracker
               CATKIN_DEPENDS people_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs message_runtime)
  
# Demo of the multivariate normal distribution
# add_executable(eigenmvn_demo src/eigenmvn_demo.cpp)
