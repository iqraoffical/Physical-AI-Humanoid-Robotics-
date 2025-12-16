cmake_minimum_required(VERSION 3.8)
project(digital_twin)

# Check if running in simulation mode
if(NOT DEFINED SIMULATION_MODE)
  set(SIMULATION_MODE ON)  # Default to simulation mode
endif()

# Set compiler standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(angles REQUIRED)

# Define message dependencies
set(MSG_DEPS
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Define message files
set(MESSAGES
  msg/JointStateMessage.msg
  msg/SensorDataMessage.msg
  msg/RobotStateMessage.msg
  msg/SimulationStateMessage.msg
)

# Define service files
set(SERVICES
  srv/LoadURDF.srv
  srv/ExecuteAction.srv
)

# Define action files
set(ACTIONS
  action/MoveRobot.action
)

# Generate message, service, and action files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGES}
  ${SERVICES}
  ${ACTIONS}
  DEPENDENCIES ${MSG_DEPS}
  ADD_LINTER_TESTS
)

# Handle Gazebo dependency conditionally
if(SIMULATION_MODE)
  # Gazebo-dependent libraries and executables
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)
  
  # Digital twin synchronization node
  add_executable(digital_twin_sync
    src/nodes/digital_twin_sync.cpp)
  ament_target_dependencies(digital_twin_sync
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    gazebo_dev
    gazebo_ros
    ${PROJECT_NAME})
  
  # Sensor processing node
  add_executable(sensor_processor
    src/nodes/sensor_processor.cpp)
  ament_target_dependencies(sensor_processor
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    ${PROJECT_NAME})
  
  # Unity bridge node
  add_executable(unity_ros_bridge
    src/nodes/unity_ros_bridge.cpp)
  ament_target_dependencies(unity_ros_bridge
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    ${PROJECT_NAME})
  
  # Install Gazebo-worlds
  install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/gazebo/worlds
    FILES_MATCHING PATTERN "*.world" PATTERN "*.sdf")
  
  # Install URDF models
  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
    FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro")
  
  # Install meshes
  install(DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
    FILES_MATCHING PATTERN "*.stl" PATTERN "*.dae" PATTERN "*.obj")
  
  # Install Unity assets
  install(DIRECTORY unity_scenes/
    DESTINATION share/${PROJECT_NAME}/unity_scenes
    FILES_MATCHING PATTERN "*.unity" PATTERN "*.prefab")
endif()

# Non-Gazebo dependent components (can run on real robot)
# Robot state processing node
add_executable(robot_state_processor
  src/nodes/robot_state_processor.cpp)
ament_target_dependencies(robot_state_processor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  ${PROJECT_NAME})

# Install executables
install(TARGETS
  robot_state_processor
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/simulation_controller.py
  scripts/sensor_simulator.py
  scripts/unity_connector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz configs
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
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

ament_package()