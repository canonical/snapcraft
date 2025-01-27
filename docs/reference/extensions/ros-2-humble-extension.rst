.. _ros-2-humble-extension:

ROS 2 Humble extension
======================

The ROS 2 Humble extension, internally referred to as ``ros2-humble``, helps fill in
common settings for software built with the  `ROS 2 Humble Hawksbill
<https://docs.ros.org/en/humble/index.html>`_ libraries.

This extension requires Snapcraft 7.3 and higher. It's only supported with the core22
base.


Included parts
--------------

The extension adds its own part to the project, which pulls in the ROS 2 build packages.

.. collapse:: Included parts

    .. code-block:: yaml

        ros2-humble/ros2-launch:
          source: /snap/snapcraft/13181/share/snapcraft/extensions/ros2
          plugin: make
          build-packages:
            - ros-humble-ros-environment
            - ros-humble-ros-workspace
            - ros-humble-ament-index-cpp
            - ros-humble-ament-index-python


Included build environment variables
------------------------------------

For the main part of the project, the extension sets the following build environment
variables.

.. collapse:: Included build environment variables

    .. code-block:: yaml

        build-environment:
          - ROS_VERSION: "2"
          - ROS_DISTRO: humble


Included runtime environment settings
-------------------------------------

For all apps that use the extension, it initializes a runtime environment required by
ROS 2 before launching the app, similar to sourcing the typical ROS 2
``local_setup.bash``.

.. collapse:: Included runtime environment settings

    .. code-block:: yaml

        environment:
          ROS_VERSION: "2"
          ROS_DISTRO: humble
          PYTHONPATH: $SNAP/opt/ros/humble/lib/python3.10/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
          ROS_HOME: $SNAP_USER_DATA/ros
        command-chain:
          - snap/command-chain/ros2-launch


Included package repositories
-----------------------------

The extension adds the `ROS 2 APT package repository
<http://repo.ros2.org/ubuntu/main>`_ build-time configuration for the snap, which
installs the necessary GPG key.

.. collapse:: Included package repositories

    .. code-block:: yaml

        package-repositories:
          - type: apt
            url: http://repo.ros2.org/ubuntu/main
            components:
              - main
            formats:
              - deb
            key-id: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
            key-server: keyserver.ubuntu.com
            suites:
              - jammy


Example expanded project file
-----------------------------

Here's an example of the result of a project file that uses a ROS 2 Humble extension. It
demonstrates the added plugs, packages, variables, and layouts that the extension adds
to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`ros2-talker-listener <example-ros-2-app-project-files>`
project.

.. collapse:: Expanded project file for ros2-talker-listener

    .. literalinclude:: ../code/extensions/ros-2-humble-extension-talker-listener-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 24-34, 39-59
