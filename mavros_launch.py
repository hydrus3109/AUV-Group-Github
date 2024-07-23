from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'mavros/startup_px4_usb_quirk': True},
                {'mavros/conn/heartbeat_rate': 1.0},
                {'mavros/conn/timeout': 10.0},
                {'mavros/conn/system_id': 1},
                {'mavros/conn/component_id': 1},
                {'mavros/conn/device': "udp://:14550@"}
            ]
        )
    ])
