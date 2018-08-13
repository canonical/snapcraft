import os
from setuptools import find_packages
from setuptools import setup

package_name = "listener_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), ["launch/talk_and_listen.py"]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["listener = nodes.listener:main"]},
)
