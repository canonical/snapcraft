.. 32983.md

.. _snapcraft-quickstart:

Snapcraft quickstart
====================

This quickstart *how-to* will guide you through the steps required to **create**, **build** and **publish** your own **snap package** with the *snapcraft* command line tool.

The purpose of this how-to is to provide Linux developers, packagers, and users with a basic and intermediate knowledge of how to package applications as snaps. It covers the following:

-  an overview of the snap ecosystem
-  a detailed explanation of the Snapcraft tool and its usage
-  the syntax and components of the build recipe for snaps (the :file:`snapcraft.yaml` file)
-  the process of uploading a built snap into the Snap Store

Here’s what you need
--------------------

-  a basic understanding of Linux and the command line
-  `Ubuntu 20.04 LTS <https://releases.ubuntu.com/20.04/>`__, or later, installed
-  10GB of free storage space
-  access to the internet

The above requirements are specific to this tutorial. Other distributions can be used, and other platforms are supported. See `Installing snapd <https://snapcraft.io/docs/installing-snapd>`__ for details.

Step-by-step guide
------------------

1. :ref:`Snapcraft installation and setup <snapcraft-installation-and-setup>`

   -  :ref:`Installation from the Snap Store <snapcraft-installation-and-setup-store>`
   -  :ref:`Repository version or snap <snapcraft-installation-and-setup-repository>`
   -  :ref:`Snapcraft LXD and Multipass backends <snapcraft-installation-and-setup-backend>`

2. :ref:`How Snapcraft builds a snap <how-snapcraft-builds-snaps>`

   -  :ref:`snapcraft.yaml <how-snapcraft-builds-snaps-snapcraft>`

      -  :ref:`Main definitions inside snapcraft.yaml <how-snapcraft-builds-snaps-definitions>`

   -  :ref:`Snapcraft build lifecycle <how-snapcraft-builds-snaps-build>`
   -  :ref:`Snapcraft build output <how-snapcraft-builds-snaps-output>`

3. :ref:`Basic snapcraft.yaml example <basic-snapcraft-yaml-example>`

   -  :ref:`Metadata <basic-snapcraft-yaml-example-metadata>`
   -  :ref:`Base <basic-snapcraft-yaml-example-base>`
   -  :ref:`Confinement <basic-snapcraft-yaml-example-confinement>`

      -  :ref:`Interfaces <basic-snapcraft-yaml-example-interfaces>`

   -  :ref:`Build definition <basic-snapcraft-yaml-example-build>`

      -  :ref:`The parts definition <basic-snapcraft-yaml-example-parts>`
      -  :ref:`The apps definition <basic-snapcraft-yaml-example-apps>`

4. :ref:`Intermediate snapcraft.yaml example <intermediate-snapcraft-yaml-example>`

   -  :ref:`adopt-info <intermediate-snapcraft-yaml-example-adopt>`
   -  :ref:`grade <intermediate-snapcraft-yaml-example-grade>`
   -  :ref:`architectures <intermediate-snapcraft-yaml-example-architectures>`
   -  :ref:`Build definition <intermediate-snapcraft-yaml-example-build>`

      -  :ref:`The parts definition <intermediate-snapcraft-yaml-example-parts>`
      -  :ref:`The apps definition <intermediate-snapcraft-yaml-example-apps>`

5. :ref:`Build and publishing example <build-and-publishing-example>`

   -  :ref:`Snap build process <build-and-publishing-example-build>`
   -  :ref:`Snap publication process <build-and-publishing-example-publish>`
   -  :ref:`Snap Store channels <build-and-publishing-example-channels>`
   -  :ref:`Next steps <build-and-publishing-example-next>`

.. toctree::
   :hidden:

   snapcraft-installation-and-setup
   how-snapcraft-builds-snaps
   basic-snapcraft-yaml-example
   intermediate-snapcraft-yaml-example
   build-and-publishing-example
