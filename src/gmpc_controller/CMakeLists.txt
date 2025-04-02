cmake_minimum_required(VERSION 3.0.2)
project(gmpc_controller)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

## Declare catkin package
catkin_package()

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include
)

## Add qpOASES library directory
link_directories(/usr/local/lib)

## Declare executable
add_executable(gmpc_controller_node src/gmpc_controller.cpp)

## Link libraries
target_link_libraries(gmpc_controller_node ${catkin_LIBRARIES} Eigen3::Eigen qpOASES)

