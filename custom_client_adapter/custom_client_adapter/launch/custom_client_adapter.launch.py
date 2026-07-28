from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('custom_client_adapter'))
    video_config_file = package_share / 'config' / 'rm_video.yaml'
    mqtt_config_file = package_share / 'config' / 'rm_mqtt.yaml'
    console_env = {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}',
    }

    return LaunchDescription(
        [
            Node(
                package='custom_client_adapter',
                executable='rm_video_node',
                name='rm_video',
                parameters=[str(video_config_file)],
                output='screen',
                output_format='{line}',
                additional_env=console_env,
            ),
            Node(
                package='custom_client_adapter',
                executable='rm_mqtt_node',
                name='rm_mqtt',
                parameters=[str(mqtt_config_file)],
                output='screen',
                output_format='{line}',
                additional_env=console_env,
            ),
        ]
    )
