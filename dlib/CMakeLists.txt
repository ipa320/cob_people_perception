cmake_minimum_required(VERSION 2.8.3)
project(dlib)

find_package(catkin REQUIRED)

add_custom_target(
    	build_dlib ALL
    		COMMAND  cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED 
			COMMENT "Building tarball"
)


add_custom_target(
	copy_dlib ALL
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/build/dlib-18.17/dlib/build/libdlib.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
	COMMENT "Copying ${PROJECT_SOURCE_DIR}/build/dlib-18.17/dlib/build/libdlib.a to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
)


catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
		dlib
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(dlib SHARED)
add_dependencies(dlib copy_dlib)
add_dependencies(copy_dlib build_dlib)
set_target_properties(dlib PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(build_dlib PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
install(TARGETS dlib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
