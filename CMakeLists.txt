cmake_minimum_required(VERSION 3.0.2)
project(ros_video)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs cv_bridge message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# include_directories(/opt/ros/noetic/include)  
link_directories(/opt/ros/noetic/lib)   # change your ros lib path

add_executable(publish src/publish.cpp)

target_link_libraries(publish ${catkin_LIBRARIES})
target_link_libraries(publish ${OpenCV_LIBS})
target_link_libraries(publish yaml-cpp)
target_link_libraries(publish cv_bridge)
