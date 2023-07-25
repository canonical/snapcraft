.. 19644.md

.. _release-notes-snapcraft-4-2:

Release notes: Snapcraft 4.2
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.2 <https://github.com/snapcore/snapcraft/releases/tag/4.2>`__.

Highlights for this release include:

* new ROS 2 (Foxy Fitzroy) support
* cmake Ninja generator with ``core20``
* improved track and channel listing

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Special thanks to the contributors that helped to make this release happen: `@GamePad64 <https://github.com/GamePad64>`__, `@Saviq <https://github.com/Saviq>`__, `@cjp256 <https://github.com/cjp256>`__, `@igorljubuncic <https://github.com/igorljubuncic>`__ and `@sergiusens <https://github.com/sergiusens>`__.

New Features
------------

ROS 2 Foxy Fitzroy extension and updated colcon plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 4.2 includes experimental support for Robot Operating System (ROS 2) `Foxy Fitzroy LTS <https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy>`__ with a new :ref:`extension <the-ros2-foxy-extension>` and colcon plugin when used with :ref:`core20 <base-snaps>`.

For example, ROS 2 applications can now be built with a :file:`snapcraft.yaml` file as simple as:

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS2 Talker/Listener Example
   description: |
     This example launches a ROS2 talker and listener.

   grade: devel
   confinement: strict
   base: core20

   parts:
     ros-demos:
       plugin: colcon
       source: https://github.com/ros2/demos.git
       source-branch: foxy
       source-subdir: demo_nodes_cpp
       build-packages: [make, gcc, g++]
       stage-packages: [ros-foxy-ros2launch]

   apps:
     ros2-talker-listener:
       command: opt/ros/foxy/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
       plugs: [network, network-bind]
       extensions: [ros2-foxy]

For a walkthrough on how to work with the plugin and extension, see https://snapcraft.io/blog/how-to-build-a-snap-using-ros-2-foxy.

Ninja file generation with cmake
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, the :ref:`cmake plugin <the-cmake-plugin>` creates a Makefile when used with with :ref:`core20 <base-snaps>` . This release adds the ``cmake-generator`` plugin property to optionally generate of a Ninja file:

.. code:: yaml

   hello:
       source: .
       plugin: cmake
       cmake-parameters:
         - -DCMAKE_INSTALL_PREFIX=/usr
       cmake-generator: Ninja

List channel tracks from Snapcraft
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can now view the available `channel tracks <https://snapcraft.io/docs/using-tracks>`__ for a given snap with the new ``snapcraft list-tracks <snap-name>`` command (or with its alias, *tracks*).

The command output shows a list of tracks together with their status, creation date, and assigned version pattern, which is required by a given snap revision to be able to release to a given track:

.. code:: bash

   Name    Status    Creation-Date    Version-Pattern
   latest  default   -                -

*Status* can be one of the following:

* default (implicit active)
* active
* hidden
* closed

Bug Fixes
---------

-  meta: detailed warnings for resolution of commands `@cjp256 <https://github.com/cjp256>`__ (`#3219 <https://github.com/snapcore/snapcraft/pull/3219>`__)
-  file utils: introduce get_host_tool_path() to find commands on host `@cjp256 <https://github.com/cjp256>`__ (`#3244 <https://github.com/snapcore/snapcraft/pull/3244>`__)
-  plugins v2: use repo.Repo not repo.Ubuntu in colcon `@cjp256 <https://github.com/cjp256>`__ (`#3257 <https://github.com/snapcore/snapcraft/pull/3257>`__)
-  remote-build: use requests.get() instead of urlopen() `@cjp256 <https://github.com/cjp256>`__ (`#3255 <https://github.com/snapcore/snapcraft/pull/3255>`__)
-  spread tests: fix classic patchelf linker regex to match all arches `@cjp256 <https://github.com/cjp256>`__ (`#3247 <https://github.com/snapcore/snapcraft/pull/3247>`__)
-  tests: restrict colcon / ros2-foxy test to amd64 & arm64 `@cjp256 <https://github.com/cjp256>`__ (`#3254 <https://github.com/snapcore/snapcraft/pull/3254>`__)
-  extensions: prepend the snapd glvnd path `@Saviq <https://github.com/Saviq>`__ (`#3253 <https://github.com/snapcore/snapcraft/pull/3253>`__)
-  build providers: honour http proxy settings for snapd `@cjp256 <https://github.com/cjp256>`__ (`#3251 <https://github.com/snapcore/snapcraft/pull/3251>`__)
-  snapcraft: use system certificates by default for https requests `@cjp256 <https://github.com/cjp256>`__ (`#3252 <https://github.com/snapcore/snapcraft/pull/3252>`__)

Specification and documentation changes
---------------------------------------

-  tiny typo fix `@igorljubuncic <https://github.com/igorljubuncic>`__ (`#3249 <https://github.com/snapcore/snapcraft/pull/3249>`__)
-  experimental ros2 extension & colcon v2 plugin `@cjp256 <https://github.com/cjp256>`__ (`#3203 <https://github.com/snapcore/snapcraft/pull/3203>`__)
