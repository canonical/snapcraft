.. _how-to-craft-an-ros-2-app:

Craft an ROS 2 app
==================

This how-to guide covers the steps, decisions, and implementation details that are
unique when crafting a `ROS 2 <https://docs.ros.org/en/rolling/index.html>`__-based
snap. We'll work through the aspects unique to ROS 2 apps by examining an existing
project.

There are four supported bases for ROS 2 -- core24, core22, core20, and core18.


.. _how-to-craft-an-ros-2-app-project-files:

Example project file for ROS 2 Talker/Listener
----------------------------------------------

.. tab-set::

    .. tab-item:: core18

        The following code comprises the project file for the `core18 version of ROS 2
        Talker/Listener <https://github.com/snapcraft-docs/ros2-talker-listener>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros2-talker-listener
                version: '0.1'
                summary: ROS 2 Talker/Listener Example
                description: |
                  This example launches a ROS 2 talker and listener.

                base: core18
                confinement: devmode

                parts:
                  ros-demos:
                    plugin: colcon
                    source: https://github.com/ros2/demos.git
                    source-branch: dashing
                    colcon-rosdistro: dashing
                    colcon-source-space: demo_nodes_cpp
                    stage-packages: [ros-dashing-ros2launch]

                apps:
                  ros2-talker-listener:
                    command: opt/ros/dashing/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py

    .. tab-item:: core20
        :sync: core20

        The following code comprises the project file for the `core20 version of ROS 2
        Talker/Listener <https://github.com/snapcraft-docs/ros2-talker-listener-core20>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros2-talker-listener
                version: '0.1'
                summary: ROS 2 Talker/Listener Example
                description: |
                  This example launches a ROS 2 talker and listener.

                confinement: devmode
                base: core20

                parts:
                  ros-demos:
                    plugin: colcon
                    source: https://github.com/ros2/demos.git
                    source-branch: foxy
                    source-subdir: demo_nodes_cpp
                    stage-packages: [ros-foxy-ros2launch]

                apps:
                  ros2-talker-listener:
                    command: opt/ros/foxy/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
                    extensions: [ros2-foxy]

    .. tab-item:: core22
        :sync: core22

        The following code comprises the project file for the `core22 version of ROS 2
        Talker/Listener <https://github.com/snapcraft-docs/ros2-talker-listener-core22>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros2-talker-listener
                version: '0.1'
                summary: ROS 2 Talker/Listener Example
                description: |
                  This example launches a ROS 2 talker and listener.

                confinement: devmode
                base: core22

                parts:
                  ros-demos:
                    plugin: colcon
                    source: https://github.com/ros2/demos.git
                    source-branch: humble
                    source-subdir: demo_nodes_cpp
                    stage-packages: [ros-humble-ros2launch]

                apps:
                  ros2-talker-listener:
                    command: opt/ros/humble/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
                    extensions: [ros2-humble]

    .. tab-item:: core24
        :sync: core24

        The following code comprises the project file for the `core24 version of ROS 2
        Talker/Listener <https://github.com/snapcraft-docs/ros2-talker-listener-core20>`_.

        .. dropdown:: Code

            .. code-block:: yaml
                :caption: snapcraft.yaml

                name: ros2-talker-listener
                version: '0.1'
                summary: ROS 2 Talker/Listener Example
                description: |
                  This example launches a ROS 2 talker and listener.

                confinement: devmode
                base: core24

                parts:
                  ros-demos:
                    plugin: colcon
                    source: https://github.com/ros2/demos.git
                    source-branch: jazzy
                    source-subdir: demo_nodes_cpp
                    stage-packages: [ros-jazzy-ros2launch]

                apps:
                  ros2-talker-listener:
                    command: ros2 launch demo_nodes_cpp talker_listener.launch.py
                    extensions: [ros2-jazzy]


Add an ROS 2 app
----------------

ROS 2 apps depend on special extensions that initialize the build- and run-time
environments.

To add an ROS 2 app:

#. Declare the general app keys, such as ``command``.
#. For ``extensions``, list the corresponding variant based on the core:

   .. list-table::
      :header-rows: 1

      * - Core
        - Extension
      * - core18
        - None
      * - core20
        - :ref:`ros2-foxy <reference-ros-2-extensions>`
      * - core22
        - :ref:`ros2-humble <reference-ros-2-extensions>`
      * - core24
        - :ref:`ros2-jazzy <reference-ros-2-extensions>`


Add a part written for ROS 2
----------------------------

ROS 1 parts are built with the :ref:`reference-colcon-plugin`.

To add an ROS 2 part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. If you're crafting for core18, set the following special keys:

   - Set ``colcon-rosdistro`` to select the ROS distribution.
   - Set ``colcon-source-space`` to the path in the source tree where colcon packages
     are stored.

#. For ``stage-packages``, list the ROS launch command as a dependency, based
   on the core:

   .. list-table::
      :header-rows: 1

      * - Core
        - Extension
      * - core18
        - ros-dashing-ros2launch
      * - core20
        - ros-foxy-ros2launch
      * - core22
        - ros-humble-ros2launch
      * - core24
        - ros-jazzy-ros2launch


Handle build issues
-------------------

The following errors can occur while building for ROS 2.


core18 and core20
~~~~~~~~~~~~~~~~~

The warnings regarding missing libraries that you might see when building your snap are
false positives. These libraries are build time dependencies only.


Share content between ROS 2 snaps
---------------------------------

The core20, core22 and core24 bases also offer the option to build your ROS snap using
the `content-sharing interface <https://snapcraft.io/docs/content-interface>`_. It
shares the ROS 2 content packages across multiple snaps, saving space and ensuring
package consistency throughout your snap build environment.

You can find more information in `ROS architectures with snaps
<https://ubuntu.com/robotics/docs/ros-architectures-with-snaps>`_ in the Canonical ROS
documentation.

Turning on content sharing requires two small changes in the project file. Here's the
difference in the project file when content sharing is enabled:

.. tab-set::

    .. tab-item:: core20
        :sync: core20

        .. code-block:: diff
            :caption: snapcraft.yaml

            source-subdir: demo_nodes_cpp
            -  stage-packages: [ros-foxy-ros2launch]

            apps:
              ros2-talker-listener:
                command: ros2 launch demo_nodes_cpp talker_listener.launch.py
            -   extensions: [ros2-foxy]
            +   extensions: [ros2-foxy-ros-base]

    .. tab-item:: core22
        :sync: core22

        .. code-block:: diff
            :caption: snapcraft.yaml

            source-subdir: demo_nodes_cpp
            -  stage-packages: [ros-humble-ros2launch]

            apps:
              ros2-talker-listener:
                command: ros2 launch demo_nodes_cpp talker_listener.launch.py
            -   extensions: [ros2-humble]
            +   extensions: [ros2-humble-ros-base]

    .. tab-item:: core24
        :sync: core24

        .. code-block:: diff
            :caption: snapcraft.yaml

            source-subdir: demo_nodes_cpp
            -  stage-packages: [ros-jazzy-ros2launch]

            apps:
              ros2-talker-listener:
                command: ros2 launch demo_nodes_cpp talker_listener.launch.py
            -   extensions: [ros2-jazzy]
            +   extensions: [ros2-jazzy-ros-base]

To turn on content sharing:

#. Remove the ``stage-packages`` key from the part. The package is already available in
   the content-sharing snap.
#. Change the ROS 2 extensions in ``extensions`` to the variant that corresponds to the
   snap's core:

   .. list-table::
      :header-rows: 1

      * - Core
        - Content extension
      * - core20
        - :ref:`ros-foxy-ros-core <reference-ros-2-content-extensions>`
      * - core22
        - :ref:`ros2-humble-ros-base <reference-ros-2-content-extensions>`
      * - core24
        - :ref:`ros2-jazzy-ros-base <reference-ros-2-content-extensions>`


Because the snap makes use of the content provided by another snap, you must connect
them through an interface before you can test the app.

To connect the snaps:

#. Run:

   .. code-block:: bash

       snap connect ros2-talker-listener:ros-foxy ros-foxy-ros-base

#. Verify that the connection is established by running:

   .. code-block:: bash

       snap connections ros2-talker-listener

   If the connection is successful, the output will show that through the content
   interface, the snap's ROS launch command is manually plugged in to the ROS base
   snap.
