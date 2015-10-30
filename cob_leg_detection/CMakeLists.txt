cmake_minimum_required(VERSION 2.8.3)
project(cob_leg_detection)

set(catkin_RUN_PACKAGES
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

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)	
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

#message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
#message("BFL library dirs:" ${BFL_LIBRARY_DIRS})


## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetection.cfg
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
INCLUDE_DIRS
	include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	orocos_kdl
)

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

## Declare a cpp executable
add_executable(leg_detection               
               src/laser_processor.cpp
               src/leg_detector.cpp 
               src/calc_leg_features.cpp)
               
## Add cmake target dependencies of the executable/library
add_dependencies(leg_detection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(leg_detection
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
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
