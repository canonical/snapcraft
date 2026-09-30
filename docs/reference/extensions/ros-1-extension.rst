.. _reference-ros-1-extension:

ROS 1 extension
===============

The ROS 1 extension, referred to internally as ``ros1-noetic``, helps fill in common
settings for software built with the  `ROS Noetic Ninjemys
<https://wiki.ros.org/noetic>`_ libraries.

This extension requires Snapcraft 7.3 or higher, and is supported with the core20 base.


Included parts
--------------

The extension adds its own part to the project, which pulls in the ROS build packages.

.. dropdown:: Included parts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        ros1-noetic-extension:
          build-packages:
            - ros-noetic-catkin
          plugin: make
          source: $SNAPCRAFT_EXTENSIONS_DIR/ros1


Included build environment variables
------------------------------------

For the main part of the project, the extension sets the following build environment
variables.

.. dropdown:: Included build environment variables

    .. code-block:: yaml
        :caption: snapcraft.yaml

        build-environment:
          - ROS_VERSION: "1"
          - ROS_DISTRO: noetic


Included runtime environment settings
-------------------------------------

For all apps that use the extension, it initializes a runtime environment required by
ROS before launching the app, similar to sourcing the typical ROS ``setup.sh`` or
``local_setup.sh``.

.. dropdown:: Included runtime environment settings

    .. code-block:: yaml
        :caption: snapcraft.yaml

        command-chain:
          - snap/command-chain/ros1-launch
        environment:
          PYTHONPATH: $SNAP/opt/ros/noetic/lib/python3.8/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
          ROS_DISTRO: noetic
          ROS_VERSION: "1"


Included package repositories
-----------------------------

The extension adds the `ROS APT package repository
<http://packages.ros.org/ros/ubuntu>`_ to the build-time configuration for the snap,
which installs the necessary GPG key.

.. dropdown:: Included package repositories

    .. code-block:: yaml
        :caption: snapcraft.yaml

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
              url: http://packages.ros.org/ros/ubuntu


Example expanded project file
-----------------------------

Here is an example of the result of a project file that uses the ROS 1 extension. It
demonstrates the added plugs, packages, variables, and layouts that the content
extensions add to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`ros2-talker-listener
<how-to-craft-an-ros-1-app-project-files>` project.

.. dropdown:: Expanded project file for ros2-talker-listener

    .. literalinclude:: code/ros-1-extension-talker-listener-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 19-26, 31-50
