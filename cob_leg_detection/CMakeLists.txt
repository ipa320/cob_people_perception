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
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetection.cfg
)

catkin_package()

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${orocos_kdl_INCLUDE_DIRS}
	${BFL_INCLUDE_DIRS}
	${BFL_INCLUDE_DIRS}/bfl
)

link_directories(${BFL_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(leg_detection
	src/laser_processor.cpp
	src/leg_detector.cpp 
	src/calc_leg_features.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(leg_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(leg_detection
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${BFL_LIBRARIES}
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

install(FILES config/trained_leg_detector.yaml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
