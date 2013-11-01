cmake_minimum_required(VERSION 2.8.3)
project(fccl_conversions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fccl_core
  fccl_msgs
  roscpp)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp eigen fccl_core fccl_msgs 
  DEPENDS Eigen orocos_kdl)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
include_directories(
  SYSTEM
  ${EIGEN_INCLUDE_DIRS})

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Conversions.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS
  -L${orocos_kdl_LIBRARY_DIRS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test 
                 test/fccl_conversions/Conversions.cpp
                 test/main.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
                        ${PROJECT_NAME} 
                        ${catkin_LIBRARIES}
                        ${orocos_kdl_LIBRARIES})
  set_target_properties(${PROJECT_NAME}-test PROPERTIES LINK_FLAGS
    -L${orocos_kdl_LIBRARY_DIRS})
endif()
