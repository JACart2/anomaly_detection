"""Expose real anomaly messages to the browser dashboard over rosbridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Launch a websocket bridge without any synthetic or backend AI nodes."""
    websocket_port = LaunchConfiguration('websocket_port')

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml',
                ]
            )
        ),
        launch_arguments={'port': websocket_port}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('websocket_port', default_value='9090'),
            rosbridge,
        ]
    )
