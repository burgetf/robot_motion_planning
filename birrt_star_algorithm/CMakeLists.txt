cmake_minimum_required(VERSION 2.8.3)
project(birrt_star_algorithm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  planner_data_structures
  planning_heuristics
  planning_world_builder
  validity_checker
  kuka_motion_control
  roscpp
  rospy
  std_msgs
  #robot_interface_definition
  tf
  rviz_visual_tools
)

#Find OpenMP Package
#find_package(OpenMP)

#For including robot_interface_definition header files
#find_package(robot_interface_definition REQUIRED)

#if(validity_checker_FOUND)
#  message (STATUS "Validity checker found")
#else(validity_checker_FOUND)
#  message (STATUS "Validity checker not found")
#endif()


#check for OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  if(MSVC90 OR MSVC10)
    if(MSVC90)
      set(OPENMP_DLL VCOMP90)
    elseif(MSVC10)
      set(OPENMP_DLL VCOMP100)
    endif(MSVC90)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
  endif(MSVC)
else(OPENMP_FOUND)
  message (STATUS "OpenMP not found")
endif()

if (MSVC)
  Set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /O2 ${SSE_FLAGS}")
else (MSVC)
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g ${SSE_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE " ${CMAKE_CXX_FLAGS} -O3 ${SSE_FLAGS}")
endif (MSVC)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


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
   LIBRARIES birrt_star_algorithm
   CATKIN_DEPENDS kuka_motion_control planning_world_builder planner_data_structures validity_checker moveit_ros_planning_interface planning_heuristics roscpp rospy std_msgs #robot_interface_definition
   DEPENDS system_lib
)


###################
## Project Files ##
###################

set(birrt_star_algorithm_files

#CPP Files
#src/run_rrt_star_algorithm.cpp
src/birrt_star.cpp

#Header files (added only for visualization in QtCreator)
include/birrt_star_algorithm/birrt_star.h

)


###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${robot_interface_definition_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${birrt_star_algorithm_files})

## Declare a cpp executable
#Executable for running RRT-star Algorithm (Omnirob)
add_executable(birrt_star_omnirob_lbr_node src/run_birrt_star_algorithm_omnirob_lbr.cpp)
add_executable(birrt_star_lbr_only_node src/run_birrt_star_algorithm_lbr_only.cpp)
add_executable(birrt_star_omnirob_base_node src/run_birrt_star_algorithm_omnirob_base.cpp)

#Executable for running RRT-star Algorithm (Robotino)
add_executable(birrt_star_robotino_node src/run_birrt_star_algorithm_robotino.cpp)

#Executable for running RRT-star Algorithm (PR2)
add_executable(birrt_star_pr2_base_node src/run_birrt_star_algorithm_pr2_base.cpp)
add_executable(birrt_star_pr2_base_arm_node src/run_birrt_star_algorithm_pr2_base_arm.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rrt_star_algorithm_node rrt_star_algorithm_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_omnirob_lbr_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_lbr_only_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_omnirob_base_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(birrt_star_robotino_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(birrt_star_pr2_base_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(birrt_star_pr2_base_arm_node ${PROJECT_NAME} ${catkin_LIBRARIES})




## Specify libraries to link a library or executable target against
# target_link_libraries(birrt_star_algorithm_node
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
# install(TARGETS birrt_star_algorithm birrt_star_algorithm_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_birrt_star_algorithm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
