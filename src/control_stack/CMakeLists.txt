cmake_minimum_required(VERSION 3.0.2)
project(control_stack)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services, and actions ##
################################################

## You can add message, service, and action files here if you have any
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate messages if you have any
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_stack
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare the C++ executables
add_executable(publisher_node src/publisher_node.cpp)
add_executable(subscriber_node src/subscriber_node.cpp)

## Link the executables against the catkin libraries
target_link_libraries(publisher_node ${catkin_LIBRARIES})
target_link_libraries(subscriber_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install executable scripts (if any)
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install executables
install(TARGETS publisher_node subscriber_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files (if any)
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#############
## Testing ##
#############

# Add gtest-based cpp test target and link libraries (if needed)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_control_stack.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add Python-based tests if needed
# catkin_add_nosetests(test)
