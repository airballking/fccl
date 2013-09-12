cmake_minimum_required(VERSION 2.8.3)
project(fccl_core)

find_package(catkin REQUIRED COMPONENTS roscpp orocos_kdl)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp orocos_kdl
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/fccl/kdl/Semantics.cpp
  src/fccl/kdl/Transform.cpp
  src/fccl/kdl/Twist.cpp
  src/fccl/kdl/InteractionMatrix.cpp
  src/fccl/kdl/JntArray.cpp
  src/fccl/kdl/Jacobian.cpp
  src/fccl/utils/Equalities.cpp
  src/fccl/utils/Hashing.cpp
  src/fccl/utils/Printing.cpp
  src/fccl/base/Constraints.cpp
  src/fccl/base/Features.cpp
  src/fccl/base/FeatureTypes.cpp
)

## Specify libraries to link a library or executable target against
# target_link_libraries(fccl_kdl_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target 
catkin_add_gtest(${PROJECT_NAME}-test 
                 test/main.cpp 
                 test/Hashing.cpp
                 test/Transform.cpp
                 test/Twist.cpp
                 test/InteractionMatrix.cpp
                 test/JntArray.cpp
                 test/Features.cpp
                 test/Constraints.cpp)

# linking against our own library and all libraries from dependencies
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
                        ${PROJECT_NAME} 
                        ${catkin_LIBRARIES})
endif()
