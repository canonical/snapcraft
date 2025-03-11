.. _install-snapcraft:

Set up Snapcraft
================

Snapcraft can be installed on all popular Linux distributions with systemd,
macOS, and Windows.

Before you begin installation and setup, make sure your system meets the
:ref:`system requirements <system-requirements>`.


Install the main components
---------------------------

A working Snapcraft setup has two components -- Snapcraft itself, and a build
provider.


Install Snapcraft
~~~~~~~~~~~~~~~~~

.. tabs::

  .. group-tab:: Linux

    If your Linux distribution has snapd installed, the easiest way to install
    Snapcraft is from the Snap Store:

    .. code:: bash

      sudo snap install snapcraft --classic


  .. group-tab:: macOS

    #. `Install Brew <https://brew.sh#install>`_.
    #. Install Snapcraft:

       .. code:: bash

        brew install snapcraft

  .. group-tab:: Windows

    #. `Install WSL2 with Ubuntu 20.04 or higher
       <https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command>`_.
    #. In WSL2, install Snapcraft:

       .. code:: bash

         sudo snap install snapcraft


Install a build provider
~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft is compatible with two build providers -- Multipass and LXD. It
relies on a build provider to host an isolated build environment, like a
sandbox. Inside this environment, software can be built and packaged as snaps
without making potentially destructive changes to the host system.

For **core22 and higher**, LXD is the default provider on Linux, and Multipass
is the default on macOS and Windows.

For **core20**, Multipass is the default provider on all platforms.


Choose a build provider
^^^^^^^^^^^^^^^^^^^^^^^

Before proceeding, you must choose a build provider.

`Multipass <https://multipass.run>`_ creates and manages virtual machine (VM)
build instances. It automates setup and teardown of cloud-style Ubuntu VMs. It
can't be used reliably on platforms that don't support nested virtualization.
For instance, it most likely won't itself run inside a VM.

`LXD <https://linuxcontainers.org/lxd/introduction>`_ creates and manages Linux
container images. It can operate inside VMs.

If the default build provider isn't fit for your snap, you can
:ref:`switch between them <select-a-build-provider>`.


Install Multipass
^^^^^^^^^^^^^^^^^

If Multipass isn't installed, on first run Snapcraft will ask you'd like to
automatically install and configure it. If you agree, it will walk you through
installation.

.. If Multipass isn't installed while running in a non-interactive mode
.. (running from a CI/CD pipeline), snapcraft will log an error and exit.

To install Multipass on its own, run:

.. code:: bash

  sudo snap install multipass

.. tip::

  With core20 snaps and Multipass, the default virtual machine is assigned 2
  CPUs and 2GB of RAM. To extend the hardware capacity, you can set the
  following environment variables to modify CPU and memory allocation, and
  improve performance:

  .. code:: bash

    export SNAPCRAFT_BUILD_ENVIRONMENT_CPU=8
    export SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY=16G


Install LXD
^^^^^^^^^^^

To install LXD:

#. Install the app:

   .. code:: bash

     sudo snap install lxd

#. Add your user account to the ``lxd`` group so you can access the tool's
   resources:

   .. code:: bash

     sudo usermod -a -G lxd $USER

#. Log out and back in to your account for the new group to become
   active. Then, check that you're a member of the group by running:

   .. code:: bash

     groups $USER

   ``lxd`` should be present in the output.

#. Finally, initialise LXD with a lightweight, default configuration:

   .. code:: bash

     lxd init --minimal

See `How to install LXD
<https://documentation.ubuntu.com/lxd/en/latest/installing/#installing>`_ in
the LXD documentation for further installation options and troubleshooting.

.. _multiple-installs:

Install multiple instances of Snapcraft
---------------------------------------

If you're installing Snapcraft as a snap, you can install multiple concurrent
versions at the same time. Doing so could come in handy if you want to test new
features in your snaps, before they arrive in a mainstream release.

To install another instance of Snapcraft:

#. Enable parallel installs in snapd:

   .. code:: bash

     sudo snap set system experimental.parallel-instances=true

#. List all the available versions of Snapcraft. For the version you're
   interested in, take note of the value in the channel column.

   .. code:: bash

     sudo snap info snapcraft

#. Install Snapcraft using the `instance key naming
   <https://snapcraft.io/docs/parallel-installs#heading--naming>`_ syntax.
   Replace ``edge`` with whichever name is appropriate for the instance, and
   ``latest/edge`` with the target channel and track:

   .. code:: bash

     sudo snap install snapcraft_edge latest/edge --classic

Whenever you want to run this parallel version of Snapcraft, invoke the
instance name of the command -- in this example, ``snapcraft_edge``.
