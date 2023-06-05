
cmake_minimum_required(VERSION 3.0)
project(range_localization)

add_compile_options(-std=c++11)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
add_definitions("-Wall -g")

find_package(catkin REQUIRED COMPONENTS
  rtls_uwb_sensor
  nlink_parser
  visualization_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  serial
  tf
  tf_conversions
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp 
)


find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories( include
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(range_node
    # src/slam_pp_node.cpp
    src/uwb_localization.cpp
    src/uwb_interface.cpp
    # src/uwb_loc_init.cpp
)
# target_link_libraries(range_node ${Boost_LIBRARIES} ${CERES_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(range_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})
