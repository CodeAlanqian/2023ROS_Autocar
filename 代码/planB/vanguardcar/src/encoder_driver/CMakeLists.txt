cmake_minimum_required(VERSION 2.8.3)
project(encoder_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  serial
  std_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf
  dynamic_reconfigure
)

catkin_package(
 LIBRARIES art_encoder_driver
 CATKIN_DEPENDS
 rospy
 std_msgs
 serial
 geometry_msgs
 tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



 add_executable(${PROJECT_NAME}_node1
 src/imu_encoder_mix.cpp)
 add_dependencies(${PROJECT_NAME}_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_node1
    ${catkin_LIBRARIES}
  )



SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")


