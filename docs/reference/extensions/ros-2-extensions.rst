.. _reference-ros-2-foxy-extension:

ROS 2 extensions
================

The ROS 2 extensions, helps fill in common settings for software built with the  `ROS 2
<https://ros.org>`_ libraries.

There are three extensions in this family, each for a different version of ROS 2.

.. list-table::

    * - Extension
      - Internal name
      - ROS 2 version
      - Snap requirements

    * - ROS 2 Foxy
      - ``ros2-foxy``
      - `ROS 2 Foxy Fitzroy <https://docs.ros.org/en/foxy/index.html>`_
      - core20

    * - ROS 2 Humble
      - ``ros2-humble``
      - `ROS 2 Humble Hawksbill <https://docs.ros.org/en/humble/index.html>`_
      - core22

    * - ROS 2 Jazzy
      - ``ros2-jazzy``
      - `ROS 2 Jazzy Jalisco <https://docs.ros.org/en/jazzy/index.html>`_
      - core24

        :ref:`Experimental extensions enabled <how-to-enable-experimental-extensions>`

All three extensions require Snapcraft 7.3 or higiher.


Included parts
--------------

The extension adds its own part to the project, which pulls in the ROS 2 build packages.

.. tabs::

    .. group-tab:: ROS 2 Foxy

        .. collapse:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                ros2-foxy-extension:
                  build-packages:
                    - ros-foxy-ros-environment
                    - ros-foxy-ros-workspace
                    - ros-foxy-ament-index-cpp
                    - ros-foxy-ament-index-python
                  plugin: make
                  source: $SNAPCRAFT_EXTENSIONS_DIR/ros2

    .. group-tab:: ROS 2 Humble

        .. collapse:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                ros2-humble/ros2-launch:
                  source: /snap/snapcraft/13181/share/snapcraft/extensions/ros2
                  plugin: make
                  build-packages:
                    - ros-humble-ros-environment
                    - ros-humble-ros-workspace
                    - ros-humble-ament-index-cpp
                    - ros-humble-ament-index-python

    .. group-tab:: ROS 2 Jazzy

        .. collapse:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                ros2-jazzy/ros2-launch:
                  source: /snap/snapcraft/13181/share/snapcraft/extensions/ros2
                  plugin: make
                  build-packages:
                    - ros-jazzy-ros-environment
                    - ros-jazzy-ros-workspace
                    - ros-jazzy-ament-index-cpp
                    - ros-jazzy-ament-index-python


Included build environment variables
------------------------------------

For the main part of the project, the extension sets the following build environment
variables.

.. tabs::

    .. group-tab:: ROS 2 Foxy

        .. collapse:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - ROS_VERSION: "2"
                  - ROS_DISTRO: foxy

    .. group-tab:: ROS 2 Humble

        .. collapse:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - ROS_VERSION: "2"
                  - ROS_DISTRO: humble

    .. group-tab:: ROS 2 Jazzy

        .. collapse:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - ROS_VERSION: "2"
                  - ROS_DISTRO: jazzy


Included runtime environment settings
-------------------------------------

For all apps that use the extension, it initializes a runtime environment required by
ROS 2 before launching the app, similar to sourcing the typical ROS 2
``local_setup.bash``.

.. tabs::

    .. group-tab:: ROS 2 Foxy

        .. collapse:: Included runtime environment settings

            .. code-block:: yaml
                :caption: snapcraft.yaml

                command-chain:
                  - snap/command-chain/ros2-launch
                environment:
                  PYTHONPATH: $SNAP/opt/ros/foxy/lib/python3.8/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
                  ROS_DISTRO: foxy
                  ROS_VERSION: "2"

    .. group-tab:: ROS 2 Humble

        .. collapse:: Included runtime environment settings

            .. code-block:: yaml
                :caption: snapcraft.yaml

                environment:
                  ROS_VERSION: "2"
                  ROS_DISTRO: humble
                  PYTHONPATH: $SNAP/opt/ros/humble/lib/python3.10/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
                  ROS_HOME: $SNAP_USER_DATA/ros
                command-chain:
                  - snap/command-chain/ros2-launch


    .. group-tab:: ROS 2 Jazzy

        .. collapse:: Included runtime environment settings

            .. code-block:: yaml
                :caption: snapcraft.yaml

                environment:
                  ROS_VERSION: "2"
                  ROS_DISTRO: jazzy
                  PYTHONPATH: $SNAP/opt/ros/jazzy/lib/python3.12/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}
                  ROS_HOME: $SNAP_USER_DATA/ros
                command-chain:
                  - snap/command-chain/ros2-launch


Included package repositories
-----------------------------

The extension adds the `ROS 2 APT package repository
<http://repo.ros2.org/ubuntu/main>`_ build-time configuration for the snap, which
installs the necessary GPG key.

.. tabs::

    .. group-tab:: ROS 2 Foxy

        .. collapse:: Included package repositories

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
                    url: http://repo.ros2.org/ubuntu/main

    .. group-tab:: ROS 2 Humble

        .. collapse:: Included package repositories

            .. code-block:: yaml
                :caption: snapcraft.yaml

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

    .. group-tab:: ROS 2 Jazzy

        .. collapse:: Included package repositories

            .. code-block:: yaml
                :caption: snapcraft.yaml

                package-repositories:
                  - type: apt
                    url: http://packages.ros.org/ros2/ubuntu
                    components:
                      - main
                    formats:
                      - deb
                    key-id: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
                    key-server: keyserver.ubuntu.com
                    suites:
                      - noble


Example expanded project file
-----------------------------

Here are examples of the result project files that use the ROS 2 extensions. They
demonstrates the added plugs, packages, variables, and layouts that the extensions add
to the project file immediately prior to build.

Each example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The files are based on the :ref:`ros2-talker-listener
<how-to-craft-an-ros-2-app-project-files>` project.

.. tabs::

    .. group-tab:: ROS 2 Foxy

        .. collapse:: Expanded project file for ros2-talker-listener

            .. literalinclude:: code/ros-2-foxy-extension-talker-listener-expanded.diff
                :language: diff
                :lines: 3-
                :emphasize-lines: 18-28, 33-52

    .. group-tab:: ROS 2 Humble

        .. collapse:: Expanded project file for ros2-talker-listener

            .. literalinclude:: code/ros-2-humble-extension-talker-listener-expanded.diff
                :language: diff
                :lines: 3-
                :emphasize-lines: 24-34, 39-59

    .. group-tab:: ROS 2 Humble

        .. collapse:: Expanded project file for ros2-talker-listener

            .. literalinclude:: code/ros-2-jazzy-extension-talker-listener-expanded.diff
                :language: diff
                :lines: 3-
                :emphasize-lines: 18-28, 33-41, 43-53
