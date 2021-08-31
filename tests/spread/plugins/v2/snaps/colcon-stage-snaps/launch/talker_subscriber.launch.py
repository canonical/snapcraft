from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="test_minimal_publisher",
                executable="test_minimal_publisher_node",
                name="test_minimal_publisher_node",
            ),
            Node(
                package="test_minimal_subscriber",
                executable="test_minimal_subscriber_node",
                name="test_minimal_subscriber_node",
                output="screen",
                emulate_tty=True,
                parameters=[{"exit_after_receive": True}],
                on_exit=Shutdown(),
            ),
        ]
    )
