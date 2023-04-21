cmake_minimum_required(VERSION 3.0.2)
project(sea_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_pcl
  grid_map_ros
  kdtree
  roscpp
  rospy
  message_generation
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  get_state.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sea_planner
  CATKIN_DEPENDS geometry_msgs grid_map_core grid_map_msgs grid_map_pcl grid_map_ros roscpp rospy message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sea_planner_node.cpp src/rtrrt.cpp src/rtrrtp.cpp src/utility.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

