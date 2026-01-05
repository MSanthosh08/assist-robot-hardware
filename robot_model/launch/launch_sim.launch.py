import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_model'

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -----------------------------
    # Gazebo Classic
    # -----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -----------------------------
    # Spawn robot into Gazebo
    # -----------------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot'
        ],
        output='screen'
    )

    # Delay spawn to ensure Gazebo + RSP are ready
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    # -----------------------------
    # Launch everything
    # -----------------------------
    return LaunchDescription([
        gazebo,
        rsp,
        delayed_spawn
    ])
