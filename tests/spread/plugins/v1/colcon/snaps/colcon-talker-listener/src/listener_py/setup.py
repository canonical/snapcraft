import os

from setuptools import setup

package_name = "listener_py"

setup(
    name=package_name,
    version="1.0.0",
    py_modules=["listener"],
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [os.path.join("launch", "talk_and_listen.launch.py")],
        ),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["listener = listener:main"]},
)
