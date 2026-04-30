from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
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
        default_value='src/anomaly_detection/system_evaluation/bags/anomaly_bag',
        description='Output rosbag path (directory + name)'
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

    collision_node = Node(
        package="navigation",
        executable="collision_avoidance_aad_log",
        name="collision_avoidance_aad_log",
        output="screen",
    )

    anomaly_node = Node(
        package="anomaly_detection",
        executable="anomaly_detection_node",
        output="screen",
    )

    # Rosbag recording
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_name'),
            '/ai_anomaly_logging'   # <-- topics only here
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        record_arg,
        # james_arg,
        bag_name_arg,
        # navigation_james,
        # navigation_madison,
        ui_node,
        collision_node,
        anomaly_node,
        bag_record
    ])


if __name__ == "__main__":
    generate_launch_description()