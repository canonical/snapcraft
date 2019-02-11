import launch
import launch_ros.actions


def generate_launch_description():
    exit_after_receive = launch.actions.DeclareLaunchArgument(
        "exit-after-receive",
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
        parameters=[
            {
                "exit-after-receive": launch.substitutions.LaunchConfiguration(
                    "exit-after-receive"
                )
            }
        ],
    )

    return launch.LaunchDescription(
        [
            exit_after_receive,
            talker,
            listener,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=listener,
                    on_exit=[
                        launch.actions.LogInfo(
                            msg="Listener exited; tearing down entire system."
                        ),
                        launch.actions.EmitEvent(event=launch.events.Shutdown()),
                    ],
                )
            ),
        ]
    )
