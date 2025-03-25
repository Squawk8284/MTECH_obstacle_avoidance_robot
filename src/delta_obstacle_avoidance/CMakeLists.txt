cmake_minimum_required(VERSION 2.8.3)
project(delta_obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(gazebo REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
  DEPENDS gazebo
)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)

add_executable(delta_drive src/delta_drive.cpp)
add_dependencies(delta_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(delta_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


#############
## Install ##
#############

install(TARGETS delta_drive
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#install(DIRECTORY launch models rviz worlds 
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_delta_obstacle_avoidance.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
