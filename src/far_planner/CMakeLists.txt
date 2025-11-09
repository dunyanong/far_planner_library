cmake_minimum_required(VERSION 2.8.3)
project(far_planner)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
# set(BUILD_STATIC_LIBS ON)
# set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visibility_graph_msg
  pcl_ros
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  tf2_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES far_planner_ros far_planner
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Create a library for your main algorithms
add_library(${PROJECT_NAME}_lib STATIC 
  src/dynamic_graph.cpp
  src/planner_visualizer.cpp
  src/utility.cpp
  src/graph_planner.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/graph_msger.cpp
  src/terrain_planner.cpp
  src/far_planner_util.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
)

## FarPlanner ROS wrapper library
set(WRAPPER_SOURCES src/far_planner_ros.cpp)
add_library(far_planner_ros SHARED ${WRAPPER_SOURCES})
target_include_directories(far_planner_ros PUBLIC 
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# CORRECT LINKING:
target_link_libraries(far_planner_ros 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
  ${PROJECT_NAME}_lib  # This links against your main code
)

## Main executable
add_executable(${PROJECT_NAME} src/far_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS far_planner_ros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES far_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
