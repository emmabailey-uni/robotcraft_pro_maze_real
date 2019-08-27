cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_maze_pro_real)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)


catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES robotcraft_maze_pro_real
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(${PROJECT_NAME}_driver src/robot_driver.cpp)
add_executable(${PROJECT_NAME}_path_following_nav src/path_following_navigation.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_driver
   ${catkin_LIBRARIES}
 )

  target_link_libraries(${PROJECT_NAME}_path_following_nav
   ${catkin_LIBRARIES}
 )
