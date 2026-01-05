from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[
            {
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True
            }
        ]
    )

    return LaunchDescription([
        rplidar_node
    ])
'''
ros2 run rplidar_ros rplidar_composition   --ros-args   -p serial_port:=/dev/serial/by-path/platform-70090000.xusb-usb-0:2.4:1.0-port0   -p serial_baudrate:=115200   -p frame_id:=laser_frame   -p angle_compensate:=true
'''