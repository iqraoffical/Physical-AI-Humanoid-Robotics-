<?xml version="1.0"?>
<!-- Configuration file for the ROS 2 Nervous System -->
<!-- Sets up parameters for security, observability, and node configuration -->

<launch>
  <!-- Parameters for the overall system -->
  <arg name="use_sim_time" default="false"/>
  
  <!-- Security configuration -->
  <arg name="security_enable" default="true"/>
  <arg name="security_keystore" default="~/ros2_ws/keys"/>
  <arg name="security_strategy" default="Enforce"/>
  
  <!-- Observability configuration -->
  <arg name="enable_logging" default="true"/>
  <arg name="enable_metrics" default="true"/>
  <arg name="enable_tracing" default="true"/>
  
  <!-- Node-specific parameters -->
  <node pkg="robot_nervous_system" exec="publisher_node.py" name="sensor_publisher_node" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="subscriber_node.py" name="joint_subscriber_node" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="service_node" name="service_node" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="action_node" name="action_node" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="python_agent_bridge.py" name="python_agent_bridge" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="security_enable" value="$(var security_enable)"/>
  </node>
  
  <!-- Observability nodes -->
  <node pkg="robot_nervous_system" exec="logging_module.py" name="logging_module" output="screen">
    <param name="enable_logging" value="$(var enable_logging)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="metrics_collector.py" name="metrics_collector" output="screen">
    <param name="enable_metrics" value="$(var enable_metrics)"/>
  </node>
  
  <node pkg="robot_nervous_system" exec="tracing_system.py" name="tracing_system" output="screen">
    <param name="enable_tracing" value="$(var enable_tracing)"/>
  </node>
  
</launch>