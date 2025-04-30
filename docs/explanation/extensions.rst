.. _explanation-extensions:

Extensions
==========

Snapcraft extensions enable snap developers to easily incorporate a set of common
requirements into a snap.

These requirements can include build and staging packages, plugs and interfaces, file
layouts and environments, and whatever other project file elements may be required to
build a functioning system.

A snap developer creating a GTK 3 application snap, for example, can use the
``gnome-3-28`` extension to expose the GTK 3 libraries to a snap at build and runtime
without the snap developer needing specific deep knowledge about GTK 3. There are
extensions for building robotics (ROS 2) applications too, including the :ref:`ROS2
Humble Extension <reference-ros-2-foxy-extension>`.

Extensions help:

* avoid repetitive tasks in the crafting process
* obviate the need for in-depth knowledge of the target software stack
* create a standard template for common application requirements
* reduce the testing and security burden, as they're tested and updated independently

For a full list of supported extensions, see the :ref:`reference-extensions` reference.
