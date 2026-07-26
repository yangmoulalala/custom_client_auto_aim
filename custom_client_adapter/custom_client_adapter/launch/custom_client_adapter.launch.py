from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rm_mqtt.launch_support import normalize_client_id


def _launch_setup(context):
    try:
        client_id = normalize_client_id(
            LaunchConfiguration('client_id').perform(context)
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    package_share = Path(get_package_share_directory('custom_client_adapter'))
    video_config_file = package_share / 'config' / 'rm_video.yaml'
    mqtt_config_file = package_share / 'config' / 'rm_mqtt.yaml'
    console_env = {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}',
    }

    return [
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
            parameters=[str(mqtt_config_file), {'mqtt.client_id': client_id}],
            output='screen',
            output_format='{line}',
            additional_env=console_env,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'client_id',
                description='Required positive integer MQTT client ID.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
