cmake_minimum_required(VERSION 2.8.3)
project(cob_openni2_tracker)

set(catkin_RUN_PACKAGES
	geometry_msgs
	sensor_msgs
	visualization_msgs
	cob_perception_msgs
	image_transport
	cv_bridge
	std_msgs
	roscpp
	roslib
	tf
	libnite2
	nodelet
	pcl_ros
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
					
########################################################
# Find OpenNI2 (installed)
########################################################
find_path(OpenNI2_INCLUDE_DIRS
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
	  
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

#########################################################
# FIND GLUT (installed)
#########################################################
find_package(GLUT REQUIRED)

add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT was not found!")
endif(NOT GLUT_FOUND)


#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL was not found!")
endif(NOT OPENGL_FOUND)


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
	ros/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
	message_runtime
	nodelet
	pcl_ros
DEPENDS
	OpenCV
	orocos_kdl
)

include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${orocos_kdl_INCLUDE_DIRS}
	${OpenNI2_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}
)


add_executable(cob_body_tracker
	ros/src/body_tracker.cpp
)
target_link_libraries(cob_body_tracker
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${OpenNI2_LIBRARIES}
	${OPENGL_LIBRARIES}
	${GLUT_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)
add_dependencies(cob_body_tracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(body_tracker_nodelet ros/src/body_tracker.cpp ros/src/body_tracker_nodelet.cpp )
target_link_libraries(body_tracker_nodelet 
	${catkin_LIBRARIES} 
	${OpenNI2_LIBRARIES} 
	${GLUT_LIBRARIES}
)
add_dependencies(body_tracker_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_custom_command(
    TARGET cob_body_tracker
    POST_BUILD
    COMMAND ${PROJECT_SOURCE_DIR}/launch/create_nite2_link.bash
)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_body_tracker
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

set(LINK_NAME $ENV{HOME}/.ros/NiTE2)
set(NITE_DIR ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/libnite2/lib/NiTE2/)
install(CODE "
		if(EXISTS ${LINK_NAME})
			message(\"Link ${LINK_NAME} to ${NITE_DIR} already exists. Re-linking ...\")
			execute_process(COMMAND rm ${LINK_NAME})
			execute_process(COMMAND ECHO Test = ${LINK_NAME})
		endif()
		execute_process(COMMAND ln -s ${NITE_DIR} ${LINK_NAME})
		message(\"Created link ${LINK_NAME} to ${NITE_DIR}.\")
	"
)
