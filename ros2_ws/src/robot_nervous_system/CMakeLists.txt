cmake_minimum_required(VERSION 3.8)
project(robot_nervous_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Define message, service, and action dependencies
set(msg_files
  "msg/JointStateMessage.msg"
  "msg/JointCommandMessage.msg"
  "msg/SensorDataMessage.msg"
  "msg/RobotConfigurationMessage.msg"
)

set(srv_files
  "srv/LoadURDF.srv"
  "srv/ExecuteAction.srv"
)

set(action_files
  "action/MoveRobot.action"
)

# Generate message, service, and action files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs trajectory_msgs
  ADD_LINTER_TESTS
)

# Add C++ executables
add_executable(service_node src/ros2_nodes/service_node.cpp)
add_executable(action_node src/ros2_nodes/action_node.cpp)

# Define dependencies for C++ executables
ament_target_dependencies(service_node
  rclcpp
  ${PROJECT_NAME})

ament_target_dependencies(action_node
  rclcpp
  rclcpp_action
  ${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  service_node
  action_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ros2_nodes/publisher_node.py
  ros2_nodes/subscriber_node.py
  ai_bridges/python_agent_bridge.py
  security/security_context.py
  observability/logging_module.py
  observability/metrics_collector.py
  observability/tracing_system.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf_models
  DESTINATION share/${PROJECT_NAME}/
)

# Install other resource files
install(DIRECTORY
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Security settings
if(ENABLE_SECURITY)
  find_package(sros2_cmake REQUIRED)
  sros2_enable_security(${PROJECT_NAME})
endif()

ament_package()