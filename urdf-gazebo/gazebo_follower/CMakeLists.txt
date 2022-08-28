cmake_minimum_required(VERSION 2.8.3)
project(gazebo_follower)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  std_srvs 
  geometry_msgs 
  nav_msgs
  dynamic_reconfigure 
  message_generation
  message_filters
)
find_package(OpenCV 2 REQUIRED)


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
##  INCLUDE_DIRS include
##  LIBRARIES my6lbot
#  LIBRARIES gazebo_ros_range
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
##  DEPENDS gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my6lbot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(my6lbot_node src/my6lbot_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my6lbot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(my6lbot_node
#   ${catkin_LIBRARIES}
# )


add_executable(Follower src/Follower.cpp)
target_link_libraries(Follower ${catkin_LIBRARIES})

add_executable(Follower_Obstaculos src/Follower_Obstaculos.cpp)
target_link_libraries(Follower_Obstaculos ${catkin_LIBRARIES})

add_executable(odometry_publisher src/odometry_publisher.cpp)
# Add a build order dependency on nav_msgs
# This ensures that nav_msgs' msg headers are built before your executable
if(nav_msgs_EXPORTED_TARGETS)
  add_dependencies(odometry_publisher ${nav_msgs_EXPORTED_TARGETS})
endif()
# Link against the catkin libraries
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})

# Install the executable
install(TARGETS odometry_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
