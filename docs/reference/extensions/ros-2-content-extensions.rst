.. _reference-ros-2-content-extensions:

ROS 2 Content extensions
========================

The ROS 2 Content extensions comprise the main ROS 2 extensions, plus additional
settings to enable `content sharing <https://snapcraft.io/docs/content-interface>`_.

These extensions are split across ROS 2 versions and content types, and are declared
with the format ``ros2-<version>-<metapackage>``. The available extensions are:

.. tab-set::

    .. tab-item:: ROS 2 Foxy

        - ``ros2-foxy-ros-core``
        - ``ros2-foxy-ros-base``
        - ``ros2-foxy-desktop``

    .. tab-item:: ROS 2 Humble

        - ``ros2-humble-ros-core``
        - ``ros2-humble-ros-base``
        - ``ros2-humble-desktop``

    .. tab-item:: ROS 2 Jazzy

        - ``ros2-jazzy-ros-core``
        - ``ros2-jazzy-ros-base``
        - ``ros2-jazzy-desktop``

These extensions require Snapcraft 8 and higher, and are experimental.


Included interface connections
------------------------------

The most important modification the content extensions make to the project file is to
connect the `content plug <https://snapcraft.io/docs/content-interface>`_ which mounts
the provider snap content at ``$SNAP/opt/ros/underlay_ws`` and defines a default
provider.


Included build environment settings
-----------------------------------

First, the content extension includes all the build settings from the corresponding ROS
2 Foxy extension.

Then, it adds stage packages for the environment, workspace, and the C++ and Python
libraries for the `ament resource index <https://github.com/ament/ament_index>`_.

It sets the ``CMAKE_SYSTEM_PREFIX_PATH`` variable to point at the mounted system path.

Lastly, it adds the build snap that provides the build-time equivalent of the default
content-sharing provider snap to ``ros-build-snaps``.


Included runtime environment settings
-------------------------------------

First, the content extension includes all the runtime variables from the corresponding
ROS 2 extension.

It then expands the runtime environment required by ROS 2 to include the ROS 2 workspace
provided by the content-sharing snap -- similar to the conventional chaining of ROS 2
workspaces.


Example expanded project file
-----------------------------

Here is an example of the result of a project file that uses a content extension for
ROS 2 Jazzy Jalisco. It demonstrates the added plugs, packages, variables, and layouts
that the content extensions add to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`ros2-talker-listener <how-to-craft-an-ros-2-app-project-files>`
project.

.. dropdown:: Expanded project file for ros2-talker-listener

    .. literalinclude:: code/ros-2-content-desktop-extension-talker-listener-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 18-38, 43-89
