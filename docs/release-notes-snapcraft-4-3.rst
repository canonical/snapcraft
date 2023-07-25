.. 20017.md

.. _release-notes-snapcraft-4-3:

Release notes: Snapcraft 4.3
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.3 <https://github.com/snapcore/snapcraft/releases/tag/4.3>`__.

Highlights for this release include:

* a new ROS 1 extension for ``core20``
* new v2 versions of the catkin and catkin-tools plugins
* the ability to set a default track for a snap
* a new ``--enable-experimental-extensions`` option for expand-extensions

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Special thanks to the contributors that helped to make this release happen: `@cjp256 <https://github.com/cjp256>`__, `@flexiondotorg <https://github.com/flexiondotorg>`__, `@kyrofa <https://github.com/kyrofa>`__ and `@sergiusens <https://github.com/sergiusens>`__.

New Features
------------

ROS 1 Extension
~~~~~~~~~~~~~~~

The new ROS 1 :ref:`extension <snapcraft-extensions>` allows you to target ``core20`` with ROS 1 *Noetic Ninjemys*, the latest (and last) ROS 1 LTS that runs on Ubuntu 20.04 LTS (Focal Fossa).

You can now build a ROS 1 application with a :file:`snapcraft.yaml` file as simple as:

.. code:: yaml

   name: catkin-noetic-hello
   version: "1.0"
   summary: hello world
   description: |
     A ROS 1 roscpp-based workspace.
   grade: stable
   confinement: strict
   base: core20

   apps:
     catkin-noetic-hello:
       command: opt/ros/noetic/lib/snapcraft_hello/snapcraft_hello
       plugs: [network, network-bind]
       extensions: [ros1-noetic]

   parts:
     hello:
       plugin: catkin
       source: .
       build-packages: [g++, make]

It’s operation and functionality is similar to the newly introduced :ref:`ROS 2 extension <the-ros2-foxy-extension>`, and the associated colcon plugin, added in :ref:`Snapcraft 4.2 <release-notes-snapcraft-4-2>`.

Set the default Channel Track
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is now possible to set a `default track <https://snapcraft.io/docs/using-tracks>`__ for your snap:

.. code:: bash

   $ snapcraft set-default-track <snap-name> 17
   Default track for '<snap-name>' set to '17'

This features complements the ability to list channel tracks, which as added in :ref:`Snapcraft 4.2 <release-notes-snapcraft-4-2>`.

New v2 plugins
~~~~~~~~~~~~~~

catkin plugin `@kyrofa <https://github.com/kyrofa>`__ (`#3268 <https://github.com/snapcore/snapcraft/pull/3268>`__) catkin-tools plugin `@kyrofa <https://github.com/kyrofa>`__ (`#3282 <https://github.com/snapcore/snapcraft/pull/3282>`__)

CLI improvements
~~~~~~~~~~~~~~~~

-  client side check for setting default tracks `@sergiusens <https://github.com/sergiusens>`__ (`#3278 <https://github.com/snapcore/snapcraft/pull/3278>`__)
-  add ``--enable-experimental-extensions`` option for expand-extensions `@cjp256 <https://github.com/cjp256>`__ (`#3266 <https://github.com/snapcore/snapcraft/pull/3266>`__)

Build providers
~~~~~~~~~~~~~~~

-  use the releases endpoint for LXD `@sergiusens <https://github.com/sergiusens>`__ (`#3271 <https://github.com/snapcore/snapcraft/pull/3271>`__)

Bug Fixes
---------

-  spread tests: remove references of core16 `@cjp256 <https://github.com/cjp256>`__ (`#3269 <https://github.com/snapcore/snapcraft/pull/3269>`__)
-  cli: ignore sudo warning when using multipass `@sergiusens <https://github.com/sergiusens>`__ (`#3275 <https://github.com/snapcore/snapcraft/pull/3275>`__)
-  schema: rename package-repository’s “deb-types” to “format” `@cjp256 <https://github.com/cjp256>`__ (`#3274 <https://github.com/snapcore/snapcraft/pull/3274>`__)
-  spread tests: lock down setuptools for plainbox `@sergiusens <https://github.com/sergiusens>`__ (`#3273 <https://github.com/snapcore/snapcraft/pull/3273>`__)
-  build providers: hide systemd setup for LXD `@sergiusens <https://github.com/sergiusens>`__ (`#3281 <https://github.com/snapcore/snapcraft/pull/3281>`__)
-  Set VDPAU_DRIVER_PATH appropriately `@flexiondotorg <https://github.com/flexiondotorg>`__ (`#3279 <https://github.com/snapcore/snapcraft/pull/3279>`__)
-  storeapi: improve to channel map docstrings `@sergiusens <https://github.com/sergiusens>`__ (`#3272 <https://github.com/snapcore/snapcraft/pull/3272>`__)
-  colcon v2 plugin: honour http(s) proxy for stage-runtime-dependencies `@cjp256 <https://github.com/cjp256>`__ (`#3265 <https://github.com/snapcore/snapcraft/pull/3265>`__)

Specifications and Documentation
--------------------------------

-  specifications: environment lifecycle `@cjp256 <https://github.com/cjp256>`__ (`#3140 <https://github.com/snapcore/snapcraft/pull/3140>`__)
