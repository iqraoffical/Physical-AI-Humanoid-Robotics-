import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # Gazebo launch arguments
    world = LaunchConfiguration('world')
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    verbose = LaunchConfiguration('verbose')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('digital_twin'),
            'worlds',
            'humanoid_world.sdf'
        ]),
        description='Choose one of the world files from `/digital_twin/worlds` directory'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start the simulation in a paused state'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch gazebo client if true'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Enable headless mode if true'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Start gzserver in debug mode using gdb'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Run gzserver and gzclient with verbose output'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'paused': paused,
            'use_sim_time': use_sim_time,
            'gui': gui,
            'headless': headless,
            'debug': debug,
            'verbose': verbose
        }.items()
    )

    # Spawn robot in gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-file', PathJoinSubstitution([
                FindPackageShare('digital_twin'),
                'models',
                'humanoid_robot',
                'model.sdf'
            ]),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        verbose_arg,
        gazebo,
        spawn_entity
    ])