cmake_minimum_required(VERSION 2.8.3)
project(libnite2)

set(catkin_RUN_PACKAGES
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	#cmake_modules
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mk roslib rospack)

add_custom_target(
    build_libnite2 ALL
    COMMAND test -e ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED || (cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED)
)

set(libraries "NiTE2")
foreach(library ${libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/common/lib/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
    	COMMENT "Copying ${PROJECT_SOURCE_DIR}/common/lib/lib${library}.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
endforeach()

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
		common/include
	LIBRARIES
		NiTE2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	common/include
)

foreach(library ${libraries})
	add_library(${library} SHARED)
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_libnite2)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
set_target_properties(build_libnite2 PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS NiTE2
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY common/lib
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN ".svn|.so" EXCLUDE
)
