<?xml version="1.0"?>
<launch>
  <!-- Digital Twin Launch File -->
  <!-- Launches the complete digital twin environment with Gazebo simulation and Unity visualization -->
  
  <!-- Arguments -->
  <arg name="use_sim_time" default="true" description="Use simulation time for all nodes"/>
  <arg name="robot_model" default="humanoid_24dof" description="Robot model to load"/>
  <arg name="world_name" default="basic_terrain" description="World to load in Gazebo"/>
  <arg name="unity_visualization" default="true" description="Enable Unity visualization"/>
  <arg name="unity_ip" default="127.0.0.1" description="IP address of Unity application"/>
  <arg name="unity_port" default="5005" description="Port for Unity communication"/>
  
  <!-- Include Gazebo simulation -->
  <include file="$(find-pkg-share robot_gazebo)/launch/simulation.launch.py">
    <arg name="world" value="$(var world_name)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>
  
  <!-- Robot State Publisher -->
  <node pkg="robot_state_publisher" 
        exec="robot_state_publisher" 
        name="robot_state_publisher">
    <param name="robot_description" 
           value="$(command 'xacro $(find-pkg-share digital_twin)/urdf/$(var robot_model).urdf.xacro')"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <!-- Joint State Publisher (for simulation) -->
  <node pkg="joint_state_publisher" 
        exec="joint_state_publisher" 
        name="joint_state_publisher">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <!-- Digital Twin Synchronization Node -->
  <node pkg="digital_twin" 
        exec="digital_twin_sync" 
        name="digital_twin_sync" 
        output="both">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="unity_ip" value="$(var unity_ip)"/>
    <param name="unity_port" value="$(var unity_port)"/>
    <param name="sync_frequency" value="60.0"/>
    <param name="max_desync_threshold" value="0.05"/>
  </node>
  
  <!-- Sensor Processing Node -->
  <node pkg="digital_twin" 
        exec="sensor_processor" 
        name="sensor_processor" 
        output="both">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="imu_topic" value="/imu/data"/>
    <param name="lidar_topic" value="/scan"/>
    <param name="camera_topic" value="/depth_camera/image_raw"/>
  </node>
  
  <!-- Unity ROS Bridge Node (if Unity visualization enabled) -->
  <group if="$(var unity_visualization)">
    <node pkg="digital_twin" 
          exec="unity_ros_bridge" 
          name="unity_ros_bridge" 
          output="both">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="unity_ip" value="$(var unity_ip)"/>
      <param name="unity_port" value="$(var unity_port)"/>
      <param name="visualization_frequency" value="30.0"/>
    </node>
  </group>
  
  <!-- Static Transform Publishers -->
  <!-- Base link to IMU -->
  <node pkg="tf2_ros"
        exec="static_transform_publisher"
        name="base_to_imu"
        args="0.0 0.0 0.1 0.0 0.0 0.0 1.0 base_link imu_link">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <!-- Base link to LiDAR -->
  <node pkg="tf2_ros"
        exec="static_transform_publisher"
        name="base_to_lidar"
        args="0.1 0.0 0.1 0.0 0.0 0.0 1.0 base_link lidar_link">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <!-- Base link to depth camera -->
  <node pkg="tf2_ros"
        exec="static_transform_publisher"
        name="base_to_camera"
        args="0.15 0.0 0.05 0.0 0.0 0.0 1.0 base_link camera_link">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <!-- RViz for additional visualization (optional) -->
  <group>
    <node pkg="rviz2" 
          exec="rviz2" 
          name="rviz2" 
          args="-d $(find-pkg-share digital_twin)/rviz/digital_twin.rviz"
          if="$(var enable_rviz)">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
    </node>
  </group>

</launch>