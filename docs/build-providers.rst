.. 4157.md

.. _build-providers:

Build providers
===============

:ref:`Snapcraft <snapcraft-overview>`, the snap-building tool, is designed to use `LXD <https://linuxcontainers.org/lxd/introduction/>`__ or `Multipass <https://multipass.run/docs>`__ to both simplify the build process and to confine the build environment.

LXD and Multipass are referred to as **providers** because they *provide* snapcraft with build environments.

Choosing a provider
-------------------

Snapcraft supports the providers LXD and Multipass. There are a few ways to declare which provider to use.

base: core22
~~~~~~~~~~~~

There are 3 ways to choose the provider. The list below is ordered by priority, so a provider specified by option #1 takes priority over the other ways to choose a provider.

1. command-line argument ``--use-lxd``
2. environmental variable ``SNAPCRAFT_BUILD_ENVIRONMENT=<provider-name>``
3. snap configuration ``snap set snapcraft provider=<provider-name>``

If a provider is not specified, then LXD is used as the default on Linux. On other systems (macOS and Windows), Multipass is used as the default.

base: core20 \| core18 \| core
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are 4 ways to choose the provider for legacy snaps. The list below is ordered by priority, so a provider specified by option #1 takes priority over the other ways to choose a provider.

1. command-line argument ``--provider=<provider-name>``
2. environmental variable ``SNAPCRAFT_BUILD_ENVIRONMENT=<provider-name>``
3. command-line argument ``--use-lxd``
4. snap configuration ``snap set snapcraft provider=<provider-name>``

If a provider is not specified, then Multipass is used as the default.

Snap configuration
~~~~~~~~~~~~~~~~~~

The snap configuration option mentioned in the sections above is a `feature of snapd <https://snapcraft.io/docs/configuration-in-snaps>`__.

To set the provider, run ``snap set snapcraft provider=<provider-name>`` where ``<provider-name>`` can be :command:`lxd` or :command:`multipass`.

To check the provider, run ``snap get snapcraft provider``.

To unset the provider, run ``snap unset snapcraft provider``.

Install LXD
-----------

If LXD is not installed, snapcraft will prompt the user if they’d like to automatically installed and configure LXD. If LXD is not installed while running in a non-interactive mode (running from a CI/CD pipeline), snapcraft will log an error and exit.

Recent non-desktop versions of Ubuntu install LXD by default - you can check whether it is installed with the following command:

.. code:: bash

   $ lxd version
   5.6

A brief overview of installation and configuration is listed below. For more details and information, see the `LXD docs <https://linuxcontainers.org/lxd/getting-started-cli/>`__.

LXD can be installed via its snap:

.. code:: bash

   $ sudo snap install lxd

Next, initialise LXD with the following command and accept all the default options unless you have specific requirements:

.. code:: bash

   $ sudo lxd init

..

   ⓘ If the system you are installing LXD onto is using a network with a ``10.x.x.x`` subnet then network create may fail. Step through the following to resolve this.

If you try to run ``lxd init`` on a system that is connected to a network with a ``10.x.x.x`` subnet, then the final step of the init\* may fail with the following error:

.. code:: text

   Error: Failed to create network 'lxdbr0': Failed to automatically find an unused IPv4 subnet, manual configuration required

To resolve this issue, you must manually create the network with the following command:

.. code:: bash

   $ sudo lxc network create lxdbr0 ipv4.address=10.0.3.1/24 ipv4.nat=true

You can then re-run ``lxd init``. When you are prompted to create a new network bridge you must respond ``no``.

.. code:: text

   Would you like to create a new network bridge (yes/no) [default=yes]? no



Group permissions
~~~~~~~~~~~~~~~~~

If you want to build snaps as a non-root user, which is advised, then you need to add your user account to the ``lxd`` group:

.. code:: bash

   $ sudo usermod -a -G lxd ${USER}

You now need to either restart your session, reboot your computer, or use ``newgrp`` to acquire the new group assignment:

.. code:: bash

   $ newgrp lxd

..

   ⓘ The *newgrp* command will start a new sub-shell (shell within a shell) with the new ``lxd`` group assigned.

Cached LXD environment
~~~~~~~~~~~~~~~~~~~~~~

Snapcraft uses caching to speed up build times with LXD. On the first run, snapcraft creates a generic build environment for LXD and saves it locally as a LXD image.

When building a new snap or after running ``snapcraft clean``, this cached image is used as a starting point for the new environment.

Install Multipass
-----------------

If Multipass is not installed, snapcraft will prompt the user if they’d like to automatically installed and configure Multipass. If Multipass is not installed while running in a non-interactive mode (running from a CI/CD pipeline), snapcraft will log an error and exit.

Multipass can be installed via it’s snap:

.. code:: bash

   $ sudo snap install multipass

With Multipass, the default virtual machine is assigned 2 CPUs and 2GB of memory. If you have the hardware capabilities, use the following environment variables to modify CPU and memory allocation to improve performance:

.. code:: bash

   $ export SNAPCRAFT_BUILD_ENVIRONMENT_CPU=8
   $ export SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY=16G

..

   ⓘ These environmental variables are not supported when building a ``core22`` snap.

Interacting with instances
--------------------------

Entering the build environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Debugging a problematic build can require entering the build environment. Snapcraft provides the commands ``--shell``, ``--shell-after``, and ``--debug`` to allow the developer to quickly enter a shell inside the build environment. See `Iterating over a build <https://snapcraft.io/docs/iterating-over-a-build>`__ for more details.

Cleaning the build environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Build environments are re-used for every build of the same snap. If the snapcraft.yaml or source is changed significantly, the environment may need to be cleaned.

A build environment can be cleaned with ``snapcraft clean``. This deletes the environment. The next time snapcraft runs, a new environment will be used.

Running ``snapcraft clean <part-name>`` only cleans directories for a particular part. The environment will still be reused.

Building manually
-----------------

These instructions are intended to be only a general guide. For further details on using LXD as a container environment, see the `LXD Documentation <https://linuxcontainers.org/lxd/>`__.

First, create and run a new container based on Ubuntu 22.04 LTS. Our example calls this container *mysnapcraft*:

.. code:: bash

   $ lxc launch ubuntu:22.04 mysnapcraft

Copy your snap’s :file:`snapcraft.yaml` file to this new container:

.. code:: bash

   $ lxc file push snap/snapcraft.yaml mysnapcraft/home/ubuntu/

Now open an interactive shell within your container and install *snapcraft*:

.. code:: bash

   $ lxc exec mysnapcraft -- /bin/bash
   $ snap install snapcraft --classic

Finally, staying within the container, start the build by running snapcraft with the ``--destructive-mode`` argument. This forces snapcraft to build the snap directly within the current host (the *mysnapcraft* LXD container):

.. code:: bash

   $ cd /home/ubuntu
   $ snapcraft --destructive-mode

You can troubleshoot the build process just as you would on the native machine. The container is persistent and will remain until stopped and deleted.

With the build complete, you can copy your new snap to your native environment with the following command:

.. code:: bash

   $ lxc file pull mysnapcraft/home/ubuntu/mysnap.snap .
