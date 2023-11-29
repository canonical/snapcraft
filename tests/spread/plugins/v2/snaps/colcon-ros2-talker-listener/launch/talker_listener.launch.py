import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="colcon-ros2-talker-listener",
                executable="talker",
                name="talker",
                output="screen",
            ),
            DeclareLaunchArgument("exit_after_receive", default_value="False"),
            launch_ros.actions.Node(
                package="colcon-ros2-talker-listener",
                executable="listener",
                name="listener",
                output="screen",
                parameters=[
                    {"exit_after_receive": LaunchConfiguration("exit_after_receive")}
                ],
                on_exit=launch.actions.Shutdown(),
            ),
        ]
    )
