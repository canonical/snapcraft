.. 20334.md

.. _installing-snapcraft:

Installing snapcraft
====================

`Snapcraft <https://snapcraft.io/snapcraft>`__ is a powerful and easy to use command line tool for building `snaps <https://snapcraft.io/docs/quickstart-guide>`__. It helps you to both build and publish snaps, test and share them locally, and keep them all updated.

For a general overview of what Snapcraft is capable of, and how to build your first snap, take a look at our :ref:`Quickstart guide <snapcraft-overview>`, and see below for installation instructions:


.. _installing-snapcraft-linux:

Install snapcraft on Linux
--------------------------

On Linux distributions `with snap support <https://snapcraft.io/docs/installing-snapd>`__, the easiest way to install *snapcraft* is via its snap:

.. code:: bash

   $ sudo snap install snapcraft --classic

The ``--classic`` argument is required because snapcraft uses :ref:`classic confinement <snap-confinement>`.

   ⓘ If you’re using an **apt** installed version of snapcraft, such as the package for `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__, you need to remove this (``sudo apt remove snapcraft``) and install snapcraft from its snap to access the latest features.


.. _installing-snapcraft-macos:

Install snapcraft on macOS
--------------------------

Snapcraft can also be installed and run on Apple’s macOS. See `Install snapcraft on macOS <https://snapcraft.io/docs/install-snapcraft-on-macos>`__ for details.

On Apple macOS Yosemite (or later), Snapcraft can be installed via `Homebrew <https://formulae.brew.sh/formula/snapcraft>`__ and used to build snaps within the macOS environment.

Prerequisites:

- Make sure you have the `Homebrew package manager <https://brew.sh/#install>`__ installed
- Download and run the `Multipass installer <https://discourse.ubuntu.com/t/installing-multipass-on-macos/8329>`__ to install Multipass

To install *snapcraft*, open ’Terminal\` and enter the following:

.. code:: bash

   $ brew install snapcraft

When the process completes, the *snapcraft* command will be installed and ready to go. See :ref:`Snapcraft overview <snapcraft-overview>` for help getting started.
