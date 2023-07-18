.. 19639.md

.. _the-ros2-foxy-extension:

The ROS2 Foxy extension
=======================

This extension helps you snap ROS 2 applications for the `Foxy Fitzroy <https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html>`__ distribution.

   ℹ This extension requires Snapcraft 7.3+.

How to use it
-------------

Add ``extensions: [ ros2-foxy ]`` to the application definition in your ``snapcraft.yaml`` file. See the :ref:`ROS 2 deployment <ros-2-deployment-with-snaps-core20>` page for a complete tutorial on how to use this extension.

.. code:: yaml

   apps:
     colcon-ros2-foxy-rlcpp-hello:
       command: opt/ros/foxy/bin/ros2 run colcon_ros2_rlcpp_hello colcon_ros2_rlcpp_hello
       extensions: [ros2-foxy]
       ...

What it does for the build-time environment
-------------------------------------------

-  Adds `ROS 2 APT package repository <http://repo.ros2.org/ubuntu/main>`__ build-time configuration for the snap, installing the necessary GPG key.
-  Adds ``ros-foxy-ros-core`` to ``build-packages``.
-  Defines the ``build-environment``:

   -  ``ROS_VERSION = 2``
   -  ``ROS_DISTRO = foxy``

What it does for the run-time environment
-----------------------------------------

-  Initializes runtime environment required by ROS 2 before launching the application, similar to sourcing the typical ROS 2 ``local_setup.bash``.

Further reading
---------------

-  For an example of this extension being used, see the *core20* section of the :ref:`ROS 2 deployment <ros-2-deployment-with-snaps-core20>` page.
-  For a complete picture of what this extension does, add it to your app definition and run ``snapcraft expand-extensions``.

..

   ℹ Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.
