from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=[get_executable_path(package_name="talker_cpp", executable_name="talker")],
        name="talker",
    )
    ld.add_process(
        cmd=[
            get_executable_path(package_name="listener_py", executable_name="listener")
        ],
        name="listener",
    )

    return ld
