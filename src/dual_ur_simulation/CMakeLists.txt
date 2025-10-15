cmake_minimum_required(VERSION 3.8)
project(dual_ur_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install rviz files if they exist
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()