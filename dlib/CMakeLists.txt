cmake_minimum_required(VERSION 2.8.3)
project(dlib_lib)

find_package(catkin REQUIRED)

message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
add_custom_target(
    	build_dlib1817 ALL
    		COMMAND  cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball && touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}/INSTALLED 
    )
message(STATUS "done tarball")

set(libraries "dlib")
foreach(library ${libraries})
	add_custom_target(
		copy_${library} ALL
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/build/dlib-18.17/dlib/build/lib${library}.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
    	COMMENT "Copying ${PROJECT_SOURCE_DIR}/common/build/lib${library}.a to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}"
	)
endforeach()

catkin_package(
	INCLUDE_DIRS
		common/include
	LIBRARIES
		dlib
	CATKIN_DEPENDS
	DEPENDS
)

include_directories(
	common/include
	/usr/include
)

foreach(library ${libraries})
	add_library(${library} SHARED)
	add_dependencies(${library} copy_${library})
	add_dependencies(copy_${library} build_dlib1817)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
set_target_properties(build_dlib1817 PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############
install(TARGETS dlib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY /usr/include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
