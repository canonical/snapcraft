.. 11650.md

.. _release-notes-snapcraft-3-4:

Release notes: Snapcraft 3.4
============================

These are the release notes for `Snapcraft 3.4 <https://github.com/snapcore/snapcraft/releases/tag/3.4>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New *core* features
-------------------

New build provider support: LXD
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`LXD <https://linuxcontainers.org/lxd/>`__ can now be used as a build provider.

To use LXD, the snapcraft lifecycle commands, *pull*, *build*, *stage* and *prime*, together with *clean* and *snapcraft* itself, need use the –use-lxd option.

.. code:: bash

   $ snapcraft --use-lxd

With LXD, you can perform many of the same operations you can when working with Multipass, such as:

* ``--shell``
* ``--shell-after``
* ``--debug``

   LXD support is currently under-construction. Future *snapcraft* releases by break storage setups, default profiles and LXD-based projects.

snapcraft try
~~~~~~~~~~~~~

When triggering builds in a clean environment, it is sometimes desirable to run ``snap try prime`` from a local prime directory.

If it hasn’t been run before, ``snapcraft try`` runs through the lifecycle up to the *prime* stage and offers the prime directory locally.

Plugins
-------

go
~~

The :ref:`go <the-go-plugin>` plugin now works more broadly when using ``classic`` confinement. This helps avoid specifying ``no-patchelf`` for parts that fail to patch correctly.

catkin
~~~~~~

The catkin plugin has been enhanced to support stage-snaps to satisfy dependencies.

A detailed write up can be found on the `Snapcraft blog <https://snapcraft.io/blog/speed-up-your-ros-snap-builds>`__.

Full list of changes
--------------------

The issues and features worked on for 3.4 can be seen on the `3.4 <https://bugs.launchpad.net/snapcraft/+milestone/3.4>`__ launchpad milestone which are reflected in the following change list:

Sergio Schvezov
~~~~~~~~~~~~~~~

-  build providers: modify the \_run signature (`#2511 <https://github.com/snapcore/snapcraft/pull/2511>`__) (LP: #1821401)
-  build providers: support for provider setup (`#2515 <https://github.com/snapcore/snapcraft/pull/2515>`__) (LP: #1821586)
-  readme: add snap store badge (`#2516 <https://github.com/snapcore/snapcraft/pull/2516>`__)
-  build providers: initial support for LXD (`#2509 <https://github.com/snapcore/snapcraft/pull/2509>`__) (LP: #1805221)
-  cli: cleanup environment detection (`#2521 <https://github.com/snapcore/snapcraft/pull/2521>`__)
-  build providers: add API for friendly instance type names (`#2522 <https://github.com/snapcore/snapcraft/pull/2522>`__)
-  snap: set core as a base (`#2520 <https://github.com/snapcore/snapcraft/pull/2520>`__)
-  ci: improve travis integration conditionals (`#2523 <https://github.com/snapcore/snapcraft/pull/2523>`__)
-  cli: snapcraft try (`#2524 <https://github.com/snapcore/snapcraft/pull/2524>`__) (LP: #1805212)
-  build providers: idempotent destroy for LXD (`#2529 <https://github.com/snapcore/snapcraft/pull/2529>`__)
-  tests: add missing pylxd Build-Depends
-  tests: restrict catking stage-snap tests arches

Claudio Matsuoka
~~~~~~~~~~~~~~~~

-  repo: handle deb package fetch error (`#2513 <https://github.com/snapcore/snapcraft/pull/2513>`__)
-  project: ensure yaml load returns a dictionary (`#2517 <https://github.com/snapcore/snapcraft/pull/2517>`__)
-  many: better handling of appstream icons (`#2512 <https://github.com/snapcore/snapcraft/pull/2512>`__) (LP: #1814898)
-  go plugin, elf: use patchelf 0.10 and relink dynamic go binaries (`#2519 <https://github.com/snapcore/snapcraft/pull/2519>`__)
   (LP: #1805205)
-  snap: use snapcraft’s 0.10 patchelf branch (`#2528 <https://github.com/snapcore/snapcraft/pull/2528>`__)
-  snap: revert to patchelf 0.9 with local patches (`#2531 <https://github.com/snapcore/snapcraft/pull/2531>`__)

adanhawth
~~~~~~~~~

-  schema: add more detail wrt numeric version errors (`#2506 <https://github.com/snapcore/snapcraft/pull/2506>`__)

Kyle Fazzari
~~~~~~~~~~~~

-  catkin plugin: check stage-snaps for ROS dependencies (`#2525 <https://github.com/snapcore/snapcraft/pull/2525>`__)


