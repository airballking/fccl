cmake_minimum_required(VERSION 2.8.3)
project(fccl_core)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  urdf 
  kdl_parser
  reflexxes_type2
  control_toolbox)

## System dependencies 
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp urdf reflexxes_type2
  DEPENDS Boost Eigen orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
include_directories(
  SYSTEM
  ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/fccl/base/Constraints.cpp
  src/fccl/base/ConstraintFunctions.cpp
  src/fccl/control/ConstraintController.cpp
  src/fccl/kdl/InteractionMatrix.cpp
  src/fccl/kdl/KinematicChain.cpp
  src/fccl/solvers/WDLSSolver.cpp
  src/fccl/utils/Equalities.cpp
  src/fccl/utils/Hashing.cpp
  src/fccl/utils/Printing.cpp
  src/fccl/utils/TransformMap.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES})

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

## Add gtest based cpp test target 
catkin_add_gtest(${PROJECT_NAME}-test 
                 test/main.cpp 
                 test/base/Features.cpp
                 test/base/JointConstraint.cpp
                 test/base/Constraints.cpp
                 test/base/ConstraintArray.cpp
                 test/control/ConstraintController.cpp
                 test/control/FSM.cpp
                 test/control/Gains.cpp
                 test/control/Interpolator.cpp
                 test/control/PID.cpp
                 test/estimation/LimitEstimator.cpp
                 test/estimation/StateEstimator.cpp
                 test/kdl/Transform.cpp
                 test/kdl/Twist.cpp
                 test/kdl/InteractionMatrix.cpp
                 test/kdl/Jacobian.cpp
                 test/kdl/JntArray.cpp
                 test/kdl/Joint.cpp
                 test/kdl/JointArray.cpp
                 test/kdl/KinematicChain.cpp
                 test/kdl/JointMappingMatrix.cpp
                 test/semantics/TransformSemantics.cpp
                 test/semantics/JntArraySemantics.cpp
                 test/semantics/JacobianSemantics.cpp
                 test/semantics/JointMappingSemantics.cpp
                 test/semantics/InteractionMatrixSemantics.cpp
                 test/semantics/FeatureSemantics.cpp
                 test/semantics/ConstraintSemantics.cpp
                 test/semantics/TwistSemantics.cpp
                 test/solvers/WeightedSolver.cpp
                 test/utils/DoubleBuffer.cpp
                 test/utils/Hashing.cpp
                 test/utils/TransformMap.cpp
                 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)

# linking against our own library and all libraries from dependencies
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
                        ${PROJECT_NAME} 
                        ${catkin_LIBRARIES}
                        ${orocos_kdl_LIBRARIES})
endif()
