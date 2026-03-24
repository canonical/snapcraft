.. _how-to-craft-an-ros-1-app:

Craft an ROS 1 app
==================

This how-to guide covers the steps, decisions, and implementation details that are
unique when crafting a `ROS 1 <https://wiki.ros.org/noetic>`_-based snap. We'll work
through the aspects unique to ROS 1 apps by examining an existing project.

There are two supported bases for ROS 1 -- core20 and core18.

.. tip::

    Often, ROS developers rely on the *devel* space of their ``catkin`` workspace. As a
    result, it's easy to forget the importance of complete install rules, such as rules
    for installing every component of the package necessary to run, or every component
    necessary to use a given library.

    The ``catkin`` packages you're building must have install rules, or else Snapcraft
    won't know which components to place into the snap. Make sure you install necessary
    binaries, libraries, header files, and launch files.


.. _how-to-craft-an-ros-1-app-project-files:

Example project file for ROS Talker/Listener
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tab-set::

    .. tab-item:: core18

        The following code comprises the project file for the `core18 version of ROS
        Talker/Listener <https://github.com/snapcraft-docs/ros-talker-listener>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros-talker-listener
                version: '0.1'
                summary: ROS Talker/Listener Example
                description: |
                  This example launches a ROS talker and listener.

                confinement: devmode
                base: core18

                parts:
                  ros-tutorials:
                    plugin: catkin
                    source: https://github.com/ros/ros_tutorials.git
                    source-branch: melodic-devel
                    source-space: roscpp_tutorials/

                apps:
                  ros-talker-listener:
                    command: roslaunch roscpp_tutorials talker_listener.launch

    .. tab-item:: core20

        The following code comprises the project file for the `core20 version of ROS
        Talker/Listener
        <https://github.com/snapcraft-docs/ros-talker-listener-core20>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros-talker-listener
                version: '0.1'
                summary: ROS Talker/Listener Example
                description: |
                  This example launches a ROS talker and listener.

                confinement: devmode
                base: core20

                parts:
                  ros-tutorials:
                    plugin: catkin
                    source: https://github.com/ros/ros_tutorials.git
                    source-branch: noetic-devel
                    catkin-packages: [roscpp_tutorials]
                    stage-packages:
                        - ros-noetic-roslaunch

                apps:
                  ros-talker-listener:
                    command: opt/ros/noetic/bin/roslaunch roscpp_tutorials talker_listener.launch
                    extensions: [ros1-noetic]


Add an ROS 1 app based on core20
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To add an ROS 1 app based on core20:

#. Declare the general app keys, such as ``command``.
#. For ``extensions``, list ``ros1-noetic``. See :ref:`reference-ros-1-extension` for a
   description of what the extension does during build.


Add a part written for ROS 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 1 parts are built with the :ref:`reference-catkin-plugin`.

To add an ROS 1 part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: catkin``.
#. If the snap is based on core20, for ``catkin-packages``, list any ROS
   package dependencies.
