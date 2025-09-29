import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('io_control'),
        'config',
        'io.yaml'
    )

    # Check if the config file exists before launching the node
    if not os.path.isfile(config):
        print(f"Config file not found at: {config}")

    return LaunchDescription([
        Node(
            package='io_control',
            executable='io_udp',
            name='io_control',
            output='screen',
            parameters=[config]
        )
    ])
