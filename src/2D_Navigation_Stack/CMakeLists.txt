cmake_minimum_required(VERSION 3.0.2)
project(2D_Navigation_Stack)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  urdf
  xacro
  tf
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 2D_Navigation_Stack
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(odom_mobile_robot mobile_odom.cpp)
#target_link_libraries(odom_mobile_robot ${catkin_LIBRARIES})

add_executable(mobile_odom src/mobile_odom.cpp)
target_link_libraries(mobile_odom ${catkin_LIBRARIES})