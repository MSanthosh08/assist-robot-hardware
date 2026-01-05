from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # 1️⃣ Restart nvargus-daemon (Jetson CSI fix)
    restart_nvargus = ExecuteProcess(
        cmd=['sudo', 'systemctl', 'restart', 'nvargus-daemon'],
        output='screen'
    )

    stereo_camera = Node(
        package='stereo_camera_driver',
        executable='stereo_camera_node',
        name='stereo_camera',
        output='screen',
        parameters=[
            {   'fps': 30,
                'left_calib': '/home/jetson/my_eg/src/robot_model/launch/left.yaml',
                'right_calib': '/home/jetson/my_eg/src/robot_model/launch/right.yaml'
            }
        ]
    )
    left_rectify = Node(
        package='image_proc',
        executable='image_proc',
        name='left_rectify',
        remappings=[
            ('image_raw', '/left/image_raw'),
            ('camera_info', '/left/camera_info'),
            ('image_rect', '/left/image_rect'),
        ],
    )

    right_rectify = Node(
        package='image_proc',
        executable='image_proc',
        name='right_rectify',
        remappings=[
            ('image_raw', '/right/image_raw'),
            ('camera_info', '/right/camera_info'),
            ('image_rect', '/right/image_rect'),
        ],
    )

    disparity = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        output='screen'
    )

    pointcloud = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        output='screen'
    )

    return LaunchDescription([
        restart_nvargus,

        # ⏱ wait 3 seconds before starting camera
        TimerAction(period=3.0, actions=[stereo_camera]),
        TimerAction(period=4.0, actions=[left_rectify, right_rectify]),
        TimerAction(period=5.0, actions=[disparity]),
        TimerAction(period=6.0, actions=[pointcloud]),

    ])
