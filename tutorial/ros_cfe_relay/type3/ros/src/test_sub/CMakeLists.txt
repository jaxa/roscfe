cmake_minimum_required(VERSION 2.8.3)
project(test_sub)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  INCLUDE_DIRS include
#  LIBRARIES robot_ctrl
  CATKIN_DEPENDS  roscpp std_msgs 
#  DEPENDS system_lib
#  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
# add_library(robot_ctrl
#   src/${PROJECT_NAME}/robot_ctrl.cpp
# )


## Declare a cpp executable
add_executable(test_sub src/test_sub.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_ctrl_node robot_ctrl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_sub ${catkin_LIBRARIES})


install(TARGETS test_sub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

