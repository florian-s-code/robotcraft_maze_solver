cmake_minimum_required(VERSION 2.8.3)
project(maze_robotcraft)

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

add_service_files(FILES
  TargetPose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_BASICsolver src/maze_BASICsolver.cpp)
add_executable(maze_WPAplanner src/maze_WPAplanner.cpp src/WPA/WPA.cpp)


target_link_libraries(maze_BASICsolver 
  ${catkin_LIBRARIES}
)
target_link_libraries(maze_WPAplanner 
  ${catkin_LIBRARIES}
)
