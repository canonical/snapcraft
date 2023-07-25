.. 13183.md

.. _release-notes-snapcraft-3-8:

Release notes: Snapcraft 3.8
============================

These are the release notes for `Snapcraft 3.8 <https://github.com/snapcore/snapcraft/releases/tag/3.8>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New *core* features
-------------------

Windows installer
~~~~~~~~~~~~~~~~~

A lot of work has been done in preparation for the imminent installer for Microsoft Windows, which means Snapcraft will soon be able to run natively on Windows.

Improved build-base support
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following snap types have been added to :ref:`build-base <base-snaps-base-snap>`:

-  kernel
-  snapd

Extensions
----------

Extensions replace :ref:`Remote parts <remote-reusable-parts>` to help developers easily incorporate a group of components with single package.

They’re used at build and run time to ensure the inclusion of any necessary build and run dependencies.

Gnome 3.28
~~~~~~~~~~

This release includes an extension to support Gnome 3.28. For discovery, run:

.. code:: bash

   $ snapcraft list-extensions

Use the ``extension`` command to read details on what an extension does:

.. code:: bash

   $ snapcraft extension gnome-3-28

The Gnome extension configures each application with the following plugs:

-  GTK3 themes
-  common icon themes
-  common sound themes
-  Gnome 3.28 runtime libraries and utilities.

For easier desktop integration, it also configures each application entry with these additional plugs:

-  :ref:`desktop <the-desktop-interface>`
-  :ref:`desktop-legacy <the-desktop-legacy-interface>`
-  :ref:`wayland <the-wayland-interface>`
-  :ref:`x11 <the-x11-interface>`

To add the Gnome extension to an existing :file:`snapcraft.yaml` file, add ``gnome-3-28`` to the ``apps`` entry that requires it.

To see how it *extends* the :file:`snapcraft.yaml` file, at the root of the project, run

.. code:: bash

   $ snapcraft expand-extensions

Plugins
-------

:ref:`colcon <the-colcon-plugin>`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We’ve added a new syntax to ignore packages:

.. code:: yaml

   - colcon-packages-ignore:
     (list of strings)
     List of colcon packages to ignore. If not specified or set to an empty
     list ([]), no packages will be ignored.

There’s also a fix to enforce parallel building hints from snapcraft.

:ref:`catkin <the-catkin-plugin>`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Alongside colcon (above), the catkin plugin also includes a fix to enforce parallel building hints from snapcraft.

:ref:`rust <the-rust-plugin>`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Support for properly building on s390x (natively) has been added.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 3.8 are reflected in the following change list:

Chris Patterson
~~~~~~~~~~~~~~~

-  elf: handle invalid elf files
-  cli: handle exception when cleaning a part with a fresh project
-  spread: fix unbound variable error
-  docs: quick init for lxd in HACKING.md
-  windows: drop cx_Freeze support in setup.py
-  cli: use absolute import paths instead of relative imports
-  requirements: update to python 3.7 for PyYaml wheel
-  requirements: uprev all OS to pexpect 4.7.0
-  requirements: add pyinstaller 3.5 for win32
-  windows: add snapcraft.ico icon
-  windows: add pyinstaller spec file to generate frozen snapcraft.exe
-  dirs: find Windows data directory for currently-known scenarios
-  lxd: conditionally import pylxd based on OS
-  windows: add inno-installer script
-  windows: add powershell script to generate self-signed certificate
-  tests: fix snapcraft command for win32 virtual env
-  appveyor: build Windows inno-installer
-  windows: add MSIX/AppX installer
-  dirs: raise SnapcraftDataDirectoryMissingError() if paths not set
-  multipass: update ProverNotFound url to https://multipass.run
-  indicators: windows fix for is_dumb_terminal
-  multipass: add installation support for windows
-  travis: use apt addon to prevent apt update issues in CLA-check
-  multipass: fix setup exception when multipass is not found in PATH
-  dirs: check for existence of required data directories

Sergio Schvezov
~~~~~~~~~~~~~~~

-  test: autopkgtest beta
-  debian: minimal deb package for autopkgtest
-  extensions: new gnome extension (#2655)
-  deltas: code cleanup
-  tests: move meta testing to its own package
-  yaml utils: move OctInt from meta
-  spread tests: minor performance improvements
-  meta: move \_errors to errors with related error classes
-  meta: decouple DesktopFile logic
-  schema: schema: build-base support for the snapd type
-  rust plugin: support for s390x
-  schema: build-base support for the kernel type
-  spread tests: update gnome extension tests
-  extensions: rename extension classes to known names
-  extensions: create the gnome-platform directory
-  extensions: improve docsting (used in the cli)
-  spread tests: fine tune arch support for autopkgtests

Anatoli Babenia
~~~~~~~~~~~~~~~

-  lifecycle: add support for building inside podman containers (#2659)
-  docker: remove snapcraft-wrapper

Jeremie Deray
~~~~~~~~~~~~~

-  catkin plugin: forward parallel build count (#2669)
-  colcon plugin: forward parallel build count (#2670)

Kyle Fazzari
~~~~~~~~~~~~

-  spread tests: install package marker into ament index
-  colcon plugin: add ability to ignore packages (#2687)

Stefano Rivera
~~~~~~~~~~~~~~

-  repo: properly handle install query for unknown apt packages (#2692)


