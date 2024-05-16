cmake_minimum_required(VERSION 3.5)
project(inmoov_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY urdf robots
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  launch_rviz.sh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
