cmake_minimum_required(VERSION 2.8.3)
project(planning_scenarios)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rrt_star_algorithm
  birrt_star_algorithm
  planning_world_builder
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#if(birrt_star_algorithm_FOUND)
#  message (STATUS "birrt_star_algorithm found")
#else(birrt_star_algorithm_FOUND)
#  message (STATUS "birrt_star_algorithm not found")
#endif()

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
#  INCLUDE_DIRS include
#  LIBRARIES planning_scenarios
   CATKIN_DEPENDS roscpp rospy  planning_world_builder birrt_star_algorithm rrt_star_algorithm
   DEPENDS system_lib
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
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(planning_scenarios
#   src/${PROJECT_NAME}/planning_scenarios.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(planning_scenarios ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#Executable for running RRT-star Algorithm
add_executable(run_start_goal_config_generator src/generateStartGoalConfigs.cpp)
add_executable(run_block_planning src/run_block_scenario.cpp)
add_executable(run_empty_world_planning src/run_empty_world_scenario.cpp)
add_executable(run_cart_narrow_corridor_planning src/run_cart_narrow_corridor_scenario.cpp)
add_executable(run_cart_three_gates_planning src/run_cart_three_gates_scenario.cpp)
add_executable(run_cart_tunnel_planning src/run_cart_tunnel_scenario.cpp)
add_executable(run_cart_two_rooms_office_planning src/run_cart_two_rooms_office_scenario.cpp)
add_executable(run_glass_delivery_planning src/run_glass_delivery_scenario.cpp)
add_executable(run_cart_parking_slot_planning src/run_cart_parking_slot_scenario.cpp)
add_executable(run_rack_planning src/run_rack_scenario.cpp)
add_executable(run_door_planning src/run_door_scenario.cpp)
#Run planner comparison on block scenario
add_executable(run_planner_comparison_block_planning src/run_planner_comparison_block_scenario.cpp)
add_executable(run_planner_comparison_glass_planning src/run_planner_comparison_glass_scenario.cpp)



## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(planning_scenarios_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(run_start_goal_config_generator ${catkin_LIBRARIES})
target_link_libraries(run_block_planning ${catkin_LIBRARIES})
target_link_libraries(run_empty_world_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_narrow_corridor_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_three_gates_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_tunnel_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_two_rooms_office_planning ${catkin_LIBRARIES})
target_link_libraries(run_glass_delivery_planning ${catkin_LIBRARIES})
target_link_libraries(run_cart_parking_slot_planning ${catkin_LIBRARIES})
target_link_libraries(run_rack_planning ${catkin_LIBRARIES})
target_link_libraries(run_door_planning ${catkin_LIBRARIES})
target_link_libraries(run_planner_comparison_block_planning ${catkin_LIBRARIES})
target_link_libraries(run_planner_comparison_glass_planning ${catkin_LIBRARIES})



