.. _ros-2-foxy-extension:

ROS 2 Foxy extension
====================

The ROS 2 Foxy extension, internally referred to as ``ros2-foxy``, helps fill in common
settings for software built with the  `ROS 2 Foxy Fitzroy
<https://docs.ros.org/en/foxy/index.html>`_ libraries.

This extension requires Snapcraft 7.3 and higher. It's only supported with the core20
base.


Included parts
--------------

The extension adds its own part to the project, which pulls in the ROS 2 build packages.

.. collapse:: Included parts

    .. code-block:: yaml

        ros2-foxy-extension:
          build-packages:
            - ros-foxy-ros-environment
            - ros-foxy-ros-workspace
            - ros-foxy-ament-index-cpp
            - ros-foxy-ament-index-python
          plugin: make
          source: $SNAPCRAFT_EXTENSIONS_DIR/ros2


Included build environment variables
------------------------------------

For the main part of the project, the extension sets the following build environment
variables.

.. collapse:: Included build environment variables

    .. code-block:: yaml

        build-environment:
          - ROS_VERSION: "2"
          - ROS_DISTRO: foxy


Included runtime environment settings
-------------------------------------

For all apps that use the extension, it initializes a runtime environment required by
ROS 2 before launching the app, similar to sourcing the typical ROS 2
``local_setup.bash``.

.. collapse:: Included runtime environment settings

    .. code-block:: yaml

        command-chain:
          - snap/command-chain/ros2-launch
        environment:
          PYTHONPATH: $SNAP/opt/ros/foxy/lib/python3.8/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
          ROS_DISTRO: foxy
          ROS_VERSION: "2"


Included package repositories
-----------------------------

The extension adds the `ROS 2 APT package repository
<http://repo.ros2.org/ubuntu/main>`_ build-time configuration for the snap, which
installs the necessary GPG key.

.. collapse:: Included package repositories

    .. code-block:: yaml

        package-repositories:
          - components:
              - main
            formats:
              - deb
            key-id: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
            key-server: keyserver.ubuntu.com
            suites:
              - focal
            type: apt
            url: http://repo.ros2.org/ubuntu/main


Example expanded project file
-----------------------------

Here's an example of the result of a project file that uses the ROS 2 Foxy extension. It
demonstrates the added plugs, packages, variables, and layouts that the extension adds
to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`ros2-talker-listener <example-ros-2-app-project-files>`
project.

.. collapse:: Expanded project file for ros2-talker-listener

    .. literalinclude:: ../code/extensions/ros-2-foxy-extension-talker-listener-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 18-28, 33-52
