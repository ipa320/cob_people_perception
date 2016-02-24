cmake_minimum_required(VERSION 2.8.3)
project(dlib_face_detection)

find_package(catkin REQUIRED COMPONENTS
  cob_perception_msgs
  cob_vision_utils
  cv_bridge
  dlib_lib
  message_generation
  pcl_ros
  roscpp
  std_msgs
  tf  
)

find_package(X11 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(BZip2)


add_message_files(
  FILES
  Rect.msg
  RectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


set(URL http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2)
set(DOWNLOAD_PATH ${PROJECT_SOURCE_DIR}/data/shape_predictor_68_face_landmarks.dat.bz2)
set(EXTRACTED_FILE shape_predictor_68_face_landmarks.dat)

if (NOT EXISTS "${DOWNLOAD_PATH}")
    file(DOWNLOAD "${URL}" "${DOWNLOAD_PATH}")
endif()

add_custom_command(
  OUTPUT "${EXTRACTED_FILE}"
  PRE_LINK
  COMMAND bunzip2 -dkf ${PROJECT_SOURCE_DIR}/data/shape_predictor_68_face_landmarks.dat.bz2
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/data
  COMMENT "Unpacking ${DOWNLOAD_PATH}"
)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${BZIP_INCLUDE_DIRS}
  include
  
)

add_library(data_file ${EXTRACTED_FILE})
set_target_properties(data_file PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(data_file ${BZIP2_LIBRARIES})

add_executable(dlib_face_detection src/dlib_face_detection.cpp include)
add_executable(head_detector src/head_detector_node.cpp src/head_detector.cpp include)
add_executable(viewer src/detection_viewer.cpp include)
add_executable(face_detection_merge src/face_detection_merge.cpp include)

target_link_libraries(dlib_face_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES} dlib data_file)
target_link_libraries(head_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(viewer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES})
target_link_libraries(face_detection_merge ${catkin_LIBRARIES})
 
set_target_properties(head_detector PROPERTIES COMPILE_FLAGS -D__LINUX__)
add_dependencies(head_detector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############
install(TARGETS dlib_face_detection head_detector viewer face_detection_merge
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

