import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('james_perception'),
        'config',
        'bottle_detector.yaml'
    )

    if not os.path.exists(config):
        raise FileNotFoundError(
            f"Config file not found: {config}\n"
            "Run: colcon build --symlink-install --packages-select james_perception"
        )

    return LaunchDescription([
        Node(
            package='james_perception',
            executable='bottle_detector_node',
            name='bottle_detector',
            output='screen',
            parameters=[config],
        ),
    ])
