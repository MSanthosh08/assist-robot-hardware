cmake_minimum_required(VERSION 3.8)
project(robot_model)

find_package(ament_cmake REQUIRED)

# Install ament resource marker (THIS is what test_pkg has implicitly)
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install robot description files
install(
  DIRECTORY description meshes launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

