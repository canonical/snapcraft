
.. _reference-catkin-plugin:

Catkin plugin
=============

The Catkin plugin is used to build `ROS 1 <https://www.ros.org/>`_ parts.

In Snapcraft 8, the Catkin plugin can be used alongside the
:ref:`reference-ros-1-extension` to build core20 snaps.


Keywords
--------

This plugin provides the following unique keys for core20 snaps.


catkin-packages
~~~~~~~~~~~~~~~
**Type**: list of strings
**Default**: all workspace packages

The Catkin packages to build. If unset, all packages in the workspace will be built.


catkin-packages-ignore
~~~~~~~~~~~~~~~~~~~~~~
**Type**: list of strings
**Default**: []

The Catkin packages to ignore. These packages will not be built or installed. If unset
or set to an empty list, no packages will be ignored.


catkin-cmake-args
~~~~~~~~~~~~~~~~~
**Type**: list of strings
**Default**: []

The arguments to pass to CMake.


Dependencies
------------

For core20 snaps, this plugin installs ``python3-rosdep``, ``rospack-tools``,
``ros-noetic-catkin``, and any other ROS packages declared in the part.


How it works
------------

The Catkin plugin is designed to work alongside the :ref:`reference-ros-1-extension`.

For core20 snaps that don't use this extension, the ``ROS_DISTRO`` environment variable
must be set to ``noetic`` with the ``build-environment`` key.

During the build step, the plugin performs the following actions:

* Install any missing project dependencies using ``rosdep``.
* Source `Catkin workspaces <http://wiki.ros.org/catkin/workspaces>`_ in any declared
  build snaps, stage snaps, and on the host system.
* Run ``catkin_make_isolated`` to build each item in the sourced workspaces, taking
  into account any plugin-specific keys declared in the part.


Example
-------

See :ref:`how-to-craft-an-ros-1-app` for an example of a snap that uses the Catkin
plugin.
