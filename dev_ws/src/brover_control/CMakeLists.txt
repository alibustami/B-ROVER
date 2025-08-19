cmake_minimum_required(VERSION 3.10)
project(brover_control)

# Core
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Message generation
find_package(rosidl_default_generators REQUIRED)
# If Control.msg uses other message types, list them here, e.g. std_msgs:
# find_package(std_msgs REQUIRED)
# find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/Control.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python executables (if any)
install(PROGRAMS
  brover_control/bridge.py
  brover_control/encoder_mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export runtime deps so others can import brover_control.msg
ament_export_dependencies(rosidl_default_runtime)

ament_package()
