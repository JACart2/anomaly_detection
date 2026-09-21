"""Launch rosbridge and synthetic anomaly traffic for dashboard testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Build the rosbridge and synthetic publisher launch description."""
    websocket_port = LaunchConfiguration('websocket_port')
    logging_topic = LaunchConfiguration('logging_topic')
    alert_topic = LaunchConfiguration('alert_topic')
    publish_period = LaunchConfiguration('publish_period')

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

    publisher = Node(
        package='tester',
        executable='dashboard_anomaly_publisher',
        name='dashboard_anomaly_test_publisher',
        output='screen',
        parameters=[
            {
                'logging_topic': logging_topic,
                'alert_topic': alert_topic,
                'publish_period': ParameterValue(
                    publish_period, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('websocket_port', default_value='9090'),
            DeclareLaunchArgument(
                'logging_topic', default_value='/ai_anomaly_logging'
            ),
            DeclareLaunchArgument('alert_topic', default_value='/aad/alerts'),
            DeclareLaunchArgument('publish_period', default_value='2.0'),
            rosbridge,
            publisher,
        ]
    )
