cmake_minimum_required(VERSION 2.8.3)
project(cob_leg_detection)

set(catkin_RUN_PACKAGES
	cob_people_tracking_filter
	cob_perception_msgs
	dynamic_reconfigure
	geometry_msgs
	image_geometry
	laser_geometry
	roscpp
	rospy
	sensor_msgs
	std_msgs
	std_srvs
	tf
	visualization_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")

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
	${catkin_BUILD_PACKAGES}
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetector.cfg
  cfg/TrainingSetCreator.cfg
)

catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${BFL_LIBRARY_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
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

catkin_package(INCLUDE_DIRS 
                  include
               CATKIN_DEPENDS 
                  people_msgs 
                  sensor_msgs 
                  std_msgs 
                  geometry_msgs 
                  visualization_msgs 
                  message_runtime
                LIBRARIES # Libraries exported from this package
                  leg_detection 
                  )

## The leg detector
add_executable(leg_detection 
               src/laser_processor.cpp
               src/leg_detector.cpp 
               src/calc_leg_features.cpp
               src/saved_feature.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(leg_detection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(leg_detection
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${BFL_LIBRARIES}
   ${BULLET_LIBRARIES}
)

## Make a library to export stuff

add_library(leg_detection
              src/laser_processor.cpp
              src/leg_detector.cpp 
              src/calc_leg_features.cpp
              src/saved_feature.cpp
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS leg_detection
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)


install(TARGETS
    leg_detector
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS
#    dual_tracker
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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
#               
## Add cmake target dependencies of the executable/library
add_dependencies(training_set_creator people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(training_set_creator
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

## Tool to convert a labeled bag file into svgs for visual verification
add_executable(labeled_bag_file_to_svgs
               src/laser_processor.cpp
               src/calc_leg_features.cpp
               src/labeled_bag_file_to_svgs.cpp)
               
## Add cmake target dependencies of the executable/library
add_dependencies(labeled_bag_file_to_svgs people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(labeled_bag_file_to_svgs
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

## The leg train detector
add_executable(train_leg_detector
               src/laser_processor.cpp
               src/train_leg_detector.cpp 
               src/calc_leg_features.cpp)
               
## Add cmake target dependencies of the executable/library
add_dependencies(train_leg_detector people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(train_leg_detector
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)