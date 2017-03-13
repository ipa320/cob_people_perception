cmake_minimum_required(VERSION 2.8.3)
project(cob_people_tracking_filter)

set(catkin_RUN_PACKAGES
	cob_perception_msgs
	geometry_msgs
	message_filters
	roscpp
	sensor_msgs
	std_msgs
	tf
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


# Look for bfl (Bayesian Filtering Library)
#find_path(BFL_INCLUDE_DIRS REQUIRED
#	NAMES bfl_constants.h
#	HINTS /opt/ros/indigo/include/bfl)
#find_library(BFL_LIBRARIES REQUIRED
#	NAMES orocos-bfl
#	HINTS /opt/ros/indigo/lib)
#message(STATUS "BFL_LIBRARIES: ${BFL_LIBRARIES}, BFL_INCLUDE_DIRS: ${BFL_INCLUDE_DIRS}, BFL_LIBRARY_DIRS: ${BFL_LIBRARY_DIRS}")

# this way is recommended by the package (even though link_directories is not encouraged by catkin_lint)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

## Declare things to be passed to dependent projects
catkin_package(
INCLUDE_DIRS
	include
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
LIBRARIES
	cob_people_tracking_filter
)

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${BFL_INCLUDE_DIRS}
)

link_directories(${BFL_LIBRARY_DIRS})

## Declare a cpp library
add_library(cob_people_tracking_filter 
	src/uniform_vector.cpp 
	src/gaussian_vector.cpp 
	src/gaussian_pos_vel.cpp 
	src/mcpdf_pos_vel.cpp
	src/mcpdf_vector.cpp
	src/sysmodel_pos_vel.cpp
	src/sysmodel_vector.cpp
	src/measmodel_pos.cpp
	src/measmodel_vector.cpp
	src/tracker_particle.cpp 
	src/tracker_kalman.cpp 
	src/detector_particle.cpp 
)
target_link_libraries(cob_people_tracking_filter
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${BFL_LIBRARIES}
)
add_dependencies(cob_people_tracking_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(cob_people_tracker src/people_tracking_node.cpp)
target_link_libraries(cob_people_tracker
	cob_people_tracking_filter
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${BFL_LIBRARIES}
)
add_dependencies(cob_people_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
install(TARGETS cob_people_tracker cob_people_tracking_filter
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
