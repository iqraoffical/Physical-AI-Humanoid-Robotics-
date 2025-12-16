from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='humanoid_24dof',
        description='Robot model to load'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='basic.world',
        description='World to load in Gazebo'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'worlds',
                world_arg
            ]),
            'verbose': 'false'
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'config',
                'robot_config.yaml'
            ]),
            {'use_sim_time': True}
        ]
    )

    # Joint state publisher for simulation
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'source_list': ['joint_states']}
        ]
    )

    # Digital twin synchronization node
    digital_twin_bridge = Node(
        package='digital_twin',
        executable='unity_ros_connector',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'config',
                'unity_connector_config.yaml'
            ]),
            {'use_sim_time': True}
        ]
    )

    # Sensor simulator node
    sensor_simulator = Node(
        package='digital_twin',
        executable='sensor_simulator',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'config',
                'sensor_config.yaml'
            ]),
            {'use_sim_time': True}
        ]
    )

    # Simulation controller node
    simulation_controller = Node(
        package='digital_twin',
        executable='simulation_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'config',
                'controller_config.yaml'
            ]),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        sim_time_arg,
        robot_model_arg,
        world_arg,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        digital_twin_bridge,
        sensor_simulator,
        simulation_controller
    ])