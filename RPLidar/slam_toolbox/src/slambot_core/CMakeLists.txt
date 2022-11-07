cmake_minimum_required(VERSION 3.5)
project(slambot_core)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY config meshes launch rviz urdf
  DESTINATION share/${PROJECT_NAME}/
 )
 
ament_package()
