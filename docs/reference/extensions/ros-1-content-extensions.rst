.. _reference-ros-1-content-extensions:

ROS 1 Content extensions
========================

The ROS 1 Content extensions comprise the main :ref:`reference-ros-1-extension`, plus
additional settings to enable `content sharing
<https://snapcraft.io/docs/content-interface>`_.

These extensions are split across content types, and are declared with the format
``ros1-noetic-<metapackage>``. The available extensions are:

- ``ros1-noetic-ros-core``
- ``ros1-noetic-ros-base``
- ``ros1-noetic-desktop``
- ``ros1-noetic-perception``
- ``ros1-noetic-robot``


Included interface connections
------------------------------

The most important modification the content extension makes to the project file is to
connect the `content plug <https://snapcraft.io/docs/content-interface>`_ which mounts
the provider snap content at ``$SNAP/opt/ros/underlay_ws`` and defines a default
provider.


Included build environment settings
-----------------------------------

First, the content extension includes all the build settings from the ROS 1 extension.

Then, it adds the ROS environment library as a stage package.

It sets the ``CMAKE_SYSTEM_PREFIX_PATH`` variable to point at the mounted system path.

Lastly, it adds the build snap that provides the build-time equivalent of the default
content-sharing provider snap to ``ros-build-snaps``.


Included runtime environment settings
-------------------------------------

First, the content extension includes all the runtime variables from the ROS 1
extension.

It then expands the runtime environment required by ROS to include the ROS workspace
provided by the content-sharing snap -- similar to the conventional chaining of ROS
workspaces.


Example expanded project file
-----------------------------

Here is an example of the result of a project file that uses an ROS 1 Content extension.
It demonstrates the added plugs, packages, variables, and layouts that the content
extensions add to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`ros1-talker-listener
<how-to-craft-an-ros-1-app-project-files>` project.

.. dropdown:: Expanded project file for ros2-talker-listener

    .. literalinclude:: code/ros-1-content-desktop-extension-talker-listener-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 18, 20-31, 36-62
