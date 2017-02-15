cmake_minimum_required(VERSION 2.8.3)
project(cob_people_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  cob_perception_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   DetectionExt.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   cob_perception_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_people_fusion
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  libs
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


## Create the hungarian lib
add_library(hungarian_fusion
              libs/libhungarian/hungarian.c
            )

## Declare a cpp executable
add_executable(cob_people_fusion 
                src/association/association.cpp
                src/association/associatorGNN.cpp
                src/detection/detection.cpp
                src/detection/detector.cpp
                src/state_pos_vel.cpp
                src/tracker.cpp
                src/fusion_node.cpp
                src/visualization/visualization_helper.cpp
                src/fusion_node_main.cpp
              )

## Add cmake target dependencies of the executable/library
add_dependencies(cob_people_fusion cob_people_fusion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cob_people_fusion
   ${catkin_LIBRARIES} hungarian_fusion yaml-cpp
)
