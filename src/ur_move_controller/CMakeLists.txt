cmake_minimum_required(VERSION 3.8)
project(ur_move_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobot.action"
  DEPENDENCIES geometry_msgs
)

# Install Python scripts directly
install(PROGRAMS
  ur_move_controller/action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_server
)

install(PROGRAMS
  ur_move_controller/test_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_client
)

# Install Python module for imports
install(DIRECTORY ur_move_controller
  DESTINATION lib/python3.12/site-packages
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()