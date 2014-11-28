cmake_minimum_required(VERSION 2.8.3)
project(cob_openni2_tracker)

set(catkin_RUN_PACKAGES
	geometry_msgs
	visualization_msgs
	cob_perception_msgs
	orocos_kdl
	roscpp
	roslib
	tf
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
#find_package(TinyXML REQUIRED)
#find_package(Boost REQUIRED COMPONENTS filesystem)
#find_package(PCL REQUIRED)
					
########################################################
# Find OpenNI2 (download)
########################################################
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
	  
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})


#########################################################
# Find Nite2
#########################################################
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS ~/git/catkin_ws/src/cob_openni2_tracker/include/NiTE-Linux-x64-2.2/Include)
find_library(Nite2_LIBRARY
	     NAMES NiTE2 
	     HINTS ~/git/catkin_ws/src/cob_openni2_tracker/include/NiTE-Linux-x64-2.2/Redist)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT was not found!")
endif(NOT GLUT_FOUND)


#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
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
	include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
	message_runtime
DEPENDS
	OpenCV
	#TinyXML
	#Boost
	#PCL
)


include_directories(
	${catkin_INCLUDEDIR}
	${OpenCV_INCLUDE_DIRS}
	${OpenNI2_INCLUDEDIR}
	${Nite2_INCLUDEDIR}
)
		    
add_executable(body_tracker src/body_tracker.cpp)

#add_dependencies(body_tracker cob_perception_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(body_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

install(TARGETS body_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
