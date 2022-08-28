cmake_minimum_required(VERSION 2.8.3)
project(gazebo_multi_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  geometry_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_multi_robot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})

#add_executable(random_walk src/random_walk.cpp)
#target_link_libraries(random_walk ${catkin_LIBRARIES})

#add_executable(random_walk2 src/random_walk2.cpp)
#target_link_libraries(random_walk2 ${catkin_LIBRARIES})

add_executable(obstaculos src/obstaculos.cpp)
target_link_libraries(obstaculos ${catkin_LIBRARIES})
