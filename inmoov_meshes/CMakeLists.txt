cmake_minimum_required(VERSION 3.5)
project(inmoov_meshes)

# Find packages
find_package(ament_cmake REQUIRED)

# Install meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Package
ament_package()
