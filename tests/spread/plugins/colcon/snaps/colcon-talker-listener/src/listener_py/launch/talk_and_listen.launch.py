import launch
import launch_ros.actions


def generate_launch_description():
    exit_after_receive = launch.actions.DeclareLaunchArgument(
        "exit_after_receive",
        default_value="false",
        description="Exit after receiving one message.",
    )

    talker = launch_ros.actions.Node(
        package="talker_cpp",
        node_executable="talker",
        node_name="talker",
        output="screen",
    )

    listener = launch_ros.actions.Node(
        package="listener_py",
        node_executable="listener",
        node_name="listener",
        output="screen",
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {
                "exit_after_receive": launch.substitutions.LaunchConfiguration(
                    "exit_after_receive"
                )
            }
        ],
    )

    return launch.LaunchDescription([exit_after_receive, talker, listener])
