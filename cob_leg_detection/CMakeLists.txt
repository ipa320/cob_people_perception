cmake_minimum_required(VERSION 2.8.3)
project(cob_leg_detection)

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
#pkg_check_modules(BULLET bullet)
#include_directories(${BULLET_INCLUDE_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  sensor_msgs
  laser_geometry
  tf
  visualization_msgs
  cob_perception_msgs
  cob_people_tracking_filter
  image_geometry
  dynamic_reconfigure
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetection.cfg
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS cob_perception_msgs
    sensor_msgs std_msgs geometry_msgs visualization_msgs rospy)

## Declare a cpp executable
add_executable(leg_detection               
               src/laser_processor.cpp
               src/leg_detector.cpp 
               src/calc_leg_features.cpp)
               



## Add cmake target dependencies of the executable/library
#add_dependencies(leg_detection cob_perception_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(leg_detection cob_perception_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(leg_detection
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} #${BULLET_LIBRARIES}
)

#add_executable(velocity_tracker src/velocity_tracker.cpp)
#target_link_libraries(velocity_tracker ${catkin_LIBRARIES})
#add_dependencies(velocity_tracker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})


#install(TARGETS ---
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#)

install(TARGETS
    leg_detection
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
