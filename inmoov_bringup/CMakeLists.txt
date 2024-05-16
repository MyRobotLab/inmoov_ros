cmake_minimum_required(VERSION 3.5)
project(inmoov_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inmoov_msgs REQUIRED)
find_package(inmoov_description REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
# find_package(micro_ros_agent REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts and make them executable
install(PROGRAMS
  nodes/joint_command_dispatcher.py
  nodes/joint_status_dispatcher.py
  nodes/motor_status_dispatcher.py
  tools/enable_manager/enable_manager.py
  tools/rviz_manager/rviz_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
