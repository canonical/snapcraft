.. 8940.md

.. _snapcraft-overview:

Snapcraft overview
==================

`Snapcraft <https://snapcraft.io/snapcraft>`__ is a powerful and easy to use command line tool for building `snaps <https://snapcraft.io/docs/quickstart-guide>`__. It helps you to:

- build and then publish your snaps on the `Snap store <https://snapcraft.io/store>`__
- use :ref:`channels, tracks and branches <channels>` to finely control updates and releases
- build and debug snaps within a confined environment
- update and iterate over new builds without rebuilding the environment
- test and share your snaps locally

On Linux distributions `with snap support <https://snapcraft.io/docs/installing-snapd>`__, the easiest way to install *snapcraft* is via its snap:

.. code:: bash

   sudo snap install snapcraft --classic

The ``--classic`` argument is required because snapcraft uses :ref:`classic confinement <snap-confinement>`.

Snapcraft can also be installed and run on Apple’s macOS. See :ref:`installing-snapcraft-macos` for details.

See below for a general overview of Snapcraft’s capabilities, and see :ref:`Creating a snap <creating-a-snap>` for a more detailed look at the process, alongside a selection of self-contained examples for some popular languages and frameworks, including :ref:`Go <go-applications>`, :ref:`Python <python-apps>` and :ref:`C/C++ <c-c-applications>`.

   ⓘ If you’re using an **apt** installed version of snapcraft, such as the package for `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__, you need to remove this (``sudo apt remove snapcraft``) and install snapcraft from its snap to access the latest features.

Working with snapcraft
----------------------

At the heart of the snapcraft build process is a file called :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`. This file describes a snap’s build dependencies and run-time requirements, it integrates remote repositories and extensions, and runs custom scripts and hooks for better integration with CI systems.

:ref:`Snapcraft 3.0 <snapcraft-release-notes>`, and later releases, are designed to use *bases* (see :ref:`Base snaps <base-snaps>`) and `LXD <https://linuxcontainers.org/lxd/introduction/>`__ or `Multipass <https://multipass.run/>`__ to both simplify the build process and to confine the build environment within a virtual machine. Confining the build in this way isolates potentially conflicting libraries and other files from your host system, and vice-versa.

Snapcraft offers a variety of options when using LXD and Multipass. See :ref:`Build options <build-options>` for details on build options and :ref:`build providers <build-providers>` for details on interacting with LXD and Multipass.


.. _snapcraft-overview-creating-snapcraft:

Creating snapcraft.yaml
~~~~~~~~~~~~~~~~~~~~~~~

To get started, run ``snapcraft init``. This creates a buildable snapcraft.yaml template within a snap sub-directory relative to your current filesystem location. If the command cannot be found, make sure ``/snap/bin`` is on your PATH.

The typical snap build process centres on iterating over the configuration of *parts*, *plugins* and *interfaces* within this snapcraft.yaml file:

-  **parts** are the raw building blocks of a snap, used to collect and build binaries and their dependencies.
-  :ref:`plugins <snapcraft-plugins>` are used within parts to better integrate projects using languages and framework.
-  :ref:`interfaces <interface-management>` enable resources from one snap to be shared with another, and with the host system.

The following lists how you might want to approach building a new snap for your application with :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`:

#. describe your application with :ref:`top-level metadata <snapcraft-top-level-metadata>`
#. use :ref:`parts metadata <snapcraft-parts-metadata>` to import and build your application and its dependencies - incorporate *plugins* within parts to easily integrate applications using specific languages and frameworks, or work with binary files directly. You can also :ref:`write your own <writing-local-plugins>` plugin. - use :ref:`plugin metadata <supported-plugins>` to locate your project, or sync with a remote repository - set build dependencies, if required, and any run-time dependencies
#. add :ref:`interface metadata <snapcraft-app-and-service-metadata>` to connect external system resources to your application


.. _snapcraft-overview-building-your-snap:

Building your snap
------------------

When you are ready to test the contents of snapcraft.yaml, simply run ``snapcraft --debug`` in the same directory where you initialised the snap.

If this is the first time you’ve built a snap with snapcraft, you will either need to have a build provider installed, or you will be prompted to install it before the build continues:

.. code:: bash

   $ snapcraft --debug
   LXD is required but not installed. Do you wish to install LXD and configure it with the defaults? [y/N]: y

The ``--debug`` argument isn’t necessary, but it helps when testing a new snapcraft.yaml.

With ``--debug``, if snapcraft encounters an error it will automatically open a shell *within* your snap’s build environment. You can then explore the build issue directly, working on your project within the *parts* directory, or the files being staged within *prime*, depending on the build stage when the error occurred.

   ⓘ See :ref:`iterating over a build <iterating-over-a-build>` for more information about the ``--debug`` flag (and the related flags ``--shell`` and ``--shell-after``).

Critically, you can update snapcraft.yaml *outside* of the build environment and run :command:`snapcraft` *within* the build environment to incorporate any external changes and continue with the build. If there are no further errors, your snap will be built.

   ⓘ See :ref:`Debugging building snaps <debugging-building-snaps>` for common problems and their solutions.

To see snapcraft build the template created by *snapcraft init*, simply run ``snapcraft --debug``:

.. code:: bash

   $ snapcraft --debug
   Launching instance...
   Executed: pull my-part
   Executed: overlay my-part
   Executed: build my-part
   Executed: stage my-part
   Executed: prime my-part
   Executed parts lifecycle
   Generated snap metadata
   Created snap package my-snap-name_0.1_amd64.snap

The build process will proceed through the :ref:`Snapcraft lifecycle <parts-lifecycle>`, installing and building your project’s dependencies, as described by your snapcraft.yaml. The time this takes will depend on the complexity of your project and the capabilities of your system.


.. _snapcraft-overview-testing:

Testing your snap locally
-------------------------

After a snap has been built, it can be installed locally with the ``--devmode`` flag, enabling your unsigned and unconfined snap to be installed:

.. code:: bash

   sudo snap install my-snap-name_0.1_amd64.snap --devmode
   my-snap-name 0.1 installed

For a more comprehensive and iterative break-down of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.

.. note::
          ⓘ To see what’s new in each release of Snapcraft, take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>`.
