cmake_minimum_required(VERSION 2.8.4)
project(dual_people_leg_tracker)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-march=native -O3 ${CMAKE_CXX_FLAGS}")

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

#DLIB include
include(./libs/dlib/cmake)
#include(./libs/dlib/CMakeLists.txt)

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
  leg_detector
  people_tracking_filter
)

## Specify additional locations of header files
include_directories(
  include
  libs
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/DualTracker.cfg
)

## Declare a cpp library
add_library(dual_people_leg_tracker
            src/advanced_tracker_particle.cpp
            src/people_particle_filter.cpp
            src/models/occlusion_model.cpp
            src/models/advanced_measmodel_pos.cpp
            src/models/advanced_sysmodel_pos_vel.cpp
            src/distributions/multivariate_gaussian_pos_vel.cpp
            src/distributions/gaussian_pos_vel_mod.cpp
            src/leg_feature.cpp
            src/people_tracker.cpp
            src/kalman/KalmanFilter.cpp
            )

catkin_package(INCLUDE_DIRS 
                  include # The exported include paths (i.e. cflags) for the package
               LIBRARIES 
                dual_people_leg_tracker
               CATKIN_DEPENDS 
                 people_msgs 
                 sensor_msgs 
                 std_msgs 
                 geometry_msgs 
                 visualization_msgs 
                 message_runtime
                 leg_detector
                 )
  
# Demo of the multivariate normal distribution
# add_executable(eigenmvn_demo src/eigenmvn_demo.cpp)

## The leg detector
add_executable(dual_tracker
                  src/dual_tracker.cpp 
               )

## Add cmake target dependencies of the executable/library
add_dependencies(dual_tracker people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

message(STATUS "DUAL TRACKER_LIBS" ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(dual_tracker dual_people_leg_tracker ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES} dlib hungarian)

########################################################
## Demos
########################################################

## Create the hungarian lib
add_library(hungarian
              libs/libhungarian/hungarian.c
            )

## The leg detector
add_executable(murty_demo
                  src/jpda/murty_demo.cpp 
               )

target_link_libraries(murty_demo hungarian)

