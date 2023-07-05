.. 32986.md

.. _snapcraft-installation-and-setup:

Snapcraft installation and setup
================================

:ref:`Snapcraft <snapcraft-overview>` is a command line tool that allows developers to build and publish their applications as snaps.

Snapcraft can be installed and used on any supported system, including various Linux distributions as well as the macOS and Windows operating systems.


.. _snapcraft-installation-and-setup-store:

Installation from the Snap Store
--------------------------------

On Linux distributions `with snap support <https://snapcraft.io/docs/installing-snapd>`__, the easiest way to install *snapcraft* is via its snap:

::

   sudo snap install snapcraft --classic

The ``--classic`` argument is required because snapcraft uses :ref:`classic confinement <snap-confinement>`.


.. _snapcraft-installation-and-setup-repository:

Repository version or snap
--------------------------

The snap version of Snapcraft will typically receive more timely updates and thus support the full range of options and capabilities, which may not be the case with a potentially outdated repository version of the tool.

For this reason, recent versions of Ubuntu will install the snap version of Snapcraft from the ``deb`` package.

A Linux system may have both the repository and snap version of Snapcraft installed at the same time. Typically, the snap version will be the preferred one, and usually the first listed command in the user’s shell command path.

The ``which`` command can be used to check which version of snapcraft is run by default:

.. code:: bash

   $ which snapcraft
   /snap/bin/snapcraft


.. _snapcraft-installation-and-setup-backend:

Snapcraft backend
-----------------

Snapcraft, regardless of the source of the installation, relies on a backend tooling to create isolated build environment instances inside which applications can be built and packaged as snaps without changing the host system. Two backends are supported:

-  `Multipass <https://multipass.run/>`__, which creates virtual machine build instances. It cannot be reliably used on platforms that do not support nested virtualization. For instance, Multipass will most likely not run inside a virtual machine itself.
-  `LXD <https://linuxcontainers.org/lxd/introduction/>`__, which creates container image build instances. It can be used inside virtual machines.

Both tools can be installed independently, or as part of the Snapcraft installation process. Snapcraft will prompt for the installation of these additional components as part of its setup. The user can accept the default configuration suggestions without any changes.
