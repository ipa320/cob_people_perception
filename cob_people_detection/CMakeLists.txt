cmake_minimum_required(VERSION 2.8.3)
project(cob_people_detection)

set(catkin_RUN_PACKAGES
  roscpp
  std_msgs
  std_srvs
  actionlib
  sensor_msgs
  actionlib_msgs
  geometry_msgs
  stereo_msgs
  image_transport
  image_geometry
  image_view
  tf
  nodelet
  dynamic_reconfigure
  pcl_ros
  cv_bridge
  cob_image_flip
  cob_vision_utils
  cob_perception_msgs
  #kdl_parser
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(Boost REQUIRED COMPONENTS signals system filesystem)
find_package(OpenCV REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(PCL REQUIRED)
#message(STATUS "PCL Include dirs: ${PCL_INCLUDE_DIRS}")


#######################################
## Declare ROS messages and services ##
#######################################


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    addData.action
    deleteData.action
    getDetections.action
    loadModel.action
    updateData.action
)

## Generate messages in the 'msg' folder
#add_message_files(
#  DIRECTORY
#    msg
#  FILES
#    PositionMeasurement.msg
#)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    captureImage.srv
    finishRecording.srv
    recognitionTrigger.srv
)

## Generate configurations for dynamic_reconfigure
generate_dynamic_reconfigure_options(cfg/sensor_message_gateway.cfg)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    std_srvs
    sensor_msgs
    actionlib_msgs
    geometry_msgs
    stereo_msgs
    cob_perception_msgs
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
    ros/include
    common/include
  LIBRARIES
    subspace_analysis
    decomposition
    face_normalizer
    face_recognizer_algorithms
    sensor_message_gateway_nodelet
  CATKIN_DEPENDS
    ${catkin_RUN_PACKAGES}
    message_runtime
  DEPENDS
    #PCL
    OpenCV
    Boost
    #orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#library for eigenvalue decomposition of asymmetric matrices
add_library(decomposition
  common/src/decomposition.cpp
)

add_library(subspace_analysis
  common/src/subspace_analysis.cpp
)
target_link_libraries(subspace_analysis
  decomposition
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
)

add_library(face_normalizer
  common/src/face_normalizer.cpp
)
target_link_libraries(face_normalizer
  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
)

add_library(face_recognizer_algorithms
  common/src/face_recognizer_algorithms.cpp
)
target_link_libraries(face_recognizer_algorithms
  subspace_analysis
)

add_executable(face_rec_alg_test
  common/src/face_recognizer_algorithms_test.cpp
)
target_link_libraries(face_rec_alg_test
  face_recognizer_algorithms
  face_normalizer
)

add_executable(face_norm_test
  common/src/face_normalizer_test.cpp
)
target_link_libraries(face_norm_test
  face_normalizer
)

add_executable(face_align_test
  common/src/face_align_test.cpp
)
target_link_libraries(face_align_test
  face_normalizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#add_executable(synth_face_test
#  common/src/synth_face_test.cpp
#)
#target_link_libraries(synth_face_test
#  face_normalizer
#  face_recognizer_algorithms
#)

add_executable(tracking_evaluator
  ros/src/tracking_eval_node.cpp
)
target_link_libraries(tracking_evaluator
  ${catkin_LIBRARIES}
  #${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${orocos_kdl_LIBRARIES}
)

add_executable(people_detection_client
  ros/src/people_detection_action_client.cpp
)
target_link_libraries(people_detection_client
  ${catkin_LIBRARIES}
)
   
add_executable(head_detector_node
  common/src/head_detector.cpp
  ros/src/head_detector_node.cpp
)
target_link_libraries(head_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
    
add_executable(face_detector_node
  common/src/face_detector.cpp
  ros/src/face_detector_node.cpp
)
target_link_libraries(face_detector_node
  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
#  ${PCL_LIBRARIES}
)

add_executable(face_recognizer_node
  common/src/abstract_face_recognizer.cpp
  common/src/face_recognizer.cpp
  ros/src/face_recognizer_node.cpp
)
target_link_libraries(face_recognizer_node
  face_normalizer
  face_recognizer_algorithms
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)    
add_custom_command(TARGET face_recognizer_node POST_BUILD COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/common/files/training_data)

add_executable(detection_tracker_node
  ros/src/detection_tracker_node.cpp
  common/src/munkres/munkres.cpp
)
target_link_libraries(detection_tracker_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(people_detection_display_node
  ros/src/people_detection_display_node.cpp
)
target_link_libraries(people_detection_display_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(face_capture_node
  common/src/abstract_face_recognizer.cpp
  common/src/face_recognizer.cpp
  ros/src/face_capture_node.cpp
)
target_link_libraries(face_capture_node
  face_normalizer
  face_recognizer_algorithms
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(sensor_message_gateway_node
  ros/src/sensor_message_gateway_node.cpp
  ros/src/sensor_message_gateway_main.cpp
)
target_link_libraries(sensor_message_gateway_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(sensor_message_gateway_nodelet
  ros/src/sensor_message_gateway_node.cpp
  ros/src/sensor_message_gateway_nodelet.cpp
)
target_link_libraries(sensor_message_gateway_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(coordinator_node
  ros/src/coordinator_node.cpp
)
target_link_libraries(coordinator_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# modified openni tracker
#add_executable(openni_tracker
#  ros/src/openni_tracker.cpp
#  ros/src/SceneDrawer.cpp
#)


# set build flags for targets
set_target_properties(people_detection_client PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(head_detector_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(face_detector_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(face_recognizer_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(detection_tracker_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(people_detection_display_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(face_capture_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(sensor_message_gateway_node PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(sensor_message_gateway_nodelet PROPERTIES COMPILE_FLAGS -D__LINUX__)
set_target_properties(coordinator_node PROPERTIES COMPILE_FLAGS -D__LINUX__)

# make sure configure headers are built before any node using them
add_dependencies(people_detection_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(head_detector_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(face_detector_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(face_recognizer_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(detection_tracker_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(people_detection_display_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(face_capture_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(sensor_message_gateway_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(sensor_message_gateway_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(coordinator_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(tracking_evaluator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_dependencies(people_detection_client ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(head_detector_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(face_detector_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(face_recognizer_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(detection_tracker_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(people_detection_display_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(face_capture_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(sensor_message_gateway_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(sensor_message_gateway_nodelet ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(coordinator_node ${cob_perception_msgs_EXPORTED_TARGETS})
#add_dependencies(tracking_evaluator ${cob_perception_msgs_EXPORTED_TARGETS})

# Check for sse support
#rosbuild_check_for_sse()
#set(SSE_FLAGS "-msse2 -mfpmath=sse")

#rosbuild_add_roslaunch_check(ros/launch)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS people_detection_client head_detector_node face_detector_node face_recognizer_node detection_tracker_node people_detection_display_node
		face_capture_node sensor_message_gateway_node sensor_message_gateway_nodelet coordinator_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)