cmake_minimum_required(VERSION 2.8.3)
project(vrep_joy)

include_directories(SYSTEM)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  joy
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(vrep_joy src/vrep_joy.cpp)
target_link_libraries(vrep_joy ${catkin_LIBRARIES})
