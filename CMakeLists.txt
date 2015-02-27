cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

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

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetector.cfg
  cfg/TrainingSetCreator.cfg
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  ClusterMsg.msg
  LabeledRangeScanMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   rosgraph_msgs
   tf2_msgs
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS people_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs message_runtime)

## The leg detector
add_executable(leg_detector 
               src/laser_processor.cpp
               src/leg_detector.cpp 
               src/calc_leg_features.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(leg_detector people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(leg_detector
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

install(TARGETS
    leg_detector
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## The training set creator
add_executable(training_set_creator 
               src/laser_processor.cpp
               src/training_set_creator.cpp 
               src/calc_leg_features.cpp)
               
## Add cmake target dependencies of the executable/library
add_dependencies(training_set_creator people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(training_set_creator
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)