from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='true',
        description='Enable rosbag recording'
    )

    # james_arg = DeclareLaunchArgument(
    #     'james',
    #     default_value='true',
    #     description='When true, the cart is James. Otherwise it is Madison.'
    # )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='/root/dev_ws/bags/anomaly_bag',
        description='Output rosbag path (directory + name)'
    )

    launch_dashboard = DeclareLaunchArgument(
        'launch_dashboard',
        default_value='true',
        description='When true, launches the dashboard viewer.'
    )

    # # Navigation (James)
    # navigation_james = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
    #     ),
    #     condition=IfCondition(LaunchConfiguration('james'))
    # )

    # # Navigation (Madison)
    # navigation_madison = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("navigation"), "/launch/navigation.launch.py"]
    #     ),
    #     condition=UnlessCondition(LaunchConfiguration('james')),
    #     launch_arguments={
    #         "cart_config_path": "./src/ai-navigation/cart_control/cart_launch/config/cart_madison.yaml"
    #     }.items()
    # )

    # Nodes
    ui_node = Node(
        package="tester",
        executable="ui_payload_processor",
        output="screen",
    )

    # collision_node = Node(
    #     package="navigation",
    #     executable="collision_avoidance_aad_log",
    #     name="collision_avoidance_aad_log",
    #     output="screen",
    # )

    anomaly_node = Node(
        package="anomaly_detection",
        executable="anomaly_detection_node",
        output="screen",
    )

    camera_context_node = Node(
        package="anomaly_detection",
        executable="camera_context_publisher",
        output="screen",
    )

    dashboard_node = Node(
        package="anomaly_detection",
        executable="aad_dashboard_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration('launch_dashboard'))
    )

    # Rosbag recording
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_name'),
            '--qos-profile-overrides-path',
            PathJoinSubstitution([
                FindPackageShare('anomaly_detection'),
                'config',
                'rosbag_qos.yaml',
            ]),
            '--topics',
            '/ai_anomaly_logging',
            '/aad/alerts',
            '/aad/decisions',
            '/aad/formatted_messages',
            '/aad/llm_called',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        record_arg,
        # james_arg,
        bag_name_arg,
        launch_dashboard,
        # navigation_james,
        # navigation_madison,
        ui_node,
        # collision_node,
        anomaly_node,
        camera_context_node,
        dashboard_node,
        bag_record
    ])


if __name__ == "__main__":
    generate_launch_description()
