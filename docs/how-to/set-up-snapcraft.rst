.. _how-to-set-up-snapcraft:

Set up Snapcraft
================

Snapcraft can be installed on all popular Linux distributions compatible with systemd,
as well as macOS and Windows.

Before you begin installation and setup, make sure your system meets the
:ref:`system requirements <reference-system-requirements>`.


Install the main components
---------------------------

A working Snapcraft setup has two components -- Snapcraft itself, and a build
provider.


Install Snapcraft
~~~~~~~~~~~~~~~~~

.. tab-set::

  .. tab-item:: Linux

    If your Linux distribution has snapd installed, the easiest way to install
    Snapcraft is from the Snap Store:

    .. code:: bash

        snap install snapcraft --classic

  .. tab-item:: macOS

    First, `install Brew <https://brew.sh#install>`_.

    Then, install Snapcraft through Brew:

    .. code:: bash

        brew install snapcraft

  .. tab-item:: Windows

    First, `install WSL2 with Ubuntu 20.04 or higher
    <https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command>`_.

    Then, install Snapcraft in WSL2:

    .. code:: bash

        snap install snapcraft --classic


Install a build provider
~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft relies on a *build provider* to create an isolated build environment, like a
sandbox. Inside this environment, software can be built and packaged as snaps without
making potentially destructive changes to the host system.

.. list-table::
    :widths: 1 3 2

    * - Build provider
      - Description
      - Default on
    * - `LXD <https://canonical.com/lxd>`_
      - Creates and manages Linux container images. It can operate inside VMs.
      - Snapcraft 7 and higher on Linux
    * - `Multipass <https://multipass.run>`_
      - Creates and manages virtual machine (VM) build instances. It automates setup and
        teardown of cloud-style Ubuntu VMs. It can't be used reliably on platforms that
        don't support nested virtualization. In other words, it most likely won't run
        inside another VM.
      - - Snapcraft 6 on Linux
        - All versions on macOS and Windows

When you first run Snapcraft, it installs the default build provider for your host
platform. If the default build provider isn't suitable, you can install the alternative
and then :ref:`switch to it <how-to-select-a-build-provider>`.


.. _how-to-set-up-snapcraft-install-lxd:

Install LXD
^^^^^^^^^^^

Install the LXD snap:

.. code:: bash

    snap install lxd

LXD has special requirements from your local user account that Snapcraft can't manage
for you during its automatic setup. Add your user account to the ``lxd`` group so you
can access the LXD daemon:

.. code:: bash

    sudo usermod -a -G lxd $USER

Log out and back in to your local user account for the new group to take effect. Then,
check that you're a member of the group by running:

.. code:: bash

    groups $USER

The list should contain ``lxd``.

Finally, initialize LXD with a lightweight configuration:

.. code:: bash

    sudo lxd init --auto

If you need help troubleshooting your LXD installation, see `How to install LXD
<https://documentation.ubuntu.com/lxd/en/latest/installing/#installing>`_ in the LXD
documentation.


.. _how-to-set-up-snapcraft-install-multipass:

Install Multipass
^^^^^^^^^^^^^^^^^

Install the Multipass snap:

.. code:: bash

    snap install multipass

.. If Multipass isn't installed while running in a non-interactive mode
.. (running from a CI/CD pipeline), snapcraft will log an error and exit.

.. tip::

  With core20 snaps and Multipass, the default virtual machine is assigned 2
  CPUs and 2GB of RAM. To extend the hardware capacity, you can set the
  following environment variables to modify CPU and memory allocation, and
  improve performance:

  .. code:: bash

    export SNAPCRAFT_BUILD_ENVIRONMENT_CPU=8
    export SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY=16G


.. _how-to-set-up-snapcraft-multiple-instances:

Install multiple instances of Snapcraft
---------------------------------------

If you're installing Snapcraft as a snap, you can install multiple concurrent
versions at the same time. Doing so could come in handy if you want to test new
features in your snaps, before they arrive in a mainstream release.

First, enable parallel installs in snapd:

.. code:: bash

    snap set system experimental.parallel-instances=true

List all the available versions of Snapcraft. For the version you're interested in, take
note of the value in the channel column.

.. code:: bash

    snap info snapcraft

Install a new instance of Snapcraft with the `instance key naming
<https://snapcraft.io/docs/explanation/how-snaps-work/parallel-installs/>`__ syntax,
replacing ``<instance>`` with whichever name is appropriate for the instance, and
``<channel>`` with the target channel and track:

.. code:: bash

    snap install snapcraft_<instance> <channel> --classic

For example, you could install the very latest official releases with:

.. code:: bash

    snap install snapcraft_edge latest/edge --classic

Whenever you want to run this parallel version of Snapcraft, invoke the instance name of
the command -- in this example, ``snapcraft_edge``.
