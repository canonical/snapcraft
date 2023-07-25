.. 12060.md

.. _defining-a-command:

Defining a command
==================

When :ref:`creating snapcraft.yaml <creating-snapcraft-yaml>` to build a new snap, its executable components are built from :ref:`parts <adding-parts>`, following the :ref:`Snapcraft lifecycle <parts-lifecycle>`, and selectively made available to the host system.

Exposing executable components
------------------------------

A snap’s executable is exposed to the host system via the top-level ``apps`` section of :file:`snapcraft.yaml`. Its name needs to correspond with part name responsible for staging the executable within your snap.

For example, here’s a complete :file:`snapcraft.yaml` file for a fully-functional snap:

.. code:: yaml

   name: os-release
   base: core20
   version: '0.2'
   summary: Outputs the contents of /etc/os-release
   description: Prints the contents of os-release
   grade: stable
   confinement: strict
   apps:
     part-os-release:
       command: bin/os-release.sh
   parts:
     part-os-release:
       plugin: dump
       source: .
       organize:
         os-release.sh: bin/

The above example creates a snap from the following simple script, called ``os-release.sh``, located within the top-level snap of the directory:

.. code:: bash

   #! /bin/sh
   cat /etc/os-release

Make sure the above script is executable by running the following command on the build host after creating the script:

.. code:: bash

   $ chmod +x os-release.sh

The ``os-release.sh`` executable is brought into the snap via ``part-os-release``, which is a :ref:`part <adding-parts>` using the :ref:`dump plugin <the-dump-plugin>`. It then uses ``organize`` to copy the ``os-release.sh`` file into the snap’s ``bin/`` directory before the top-level :ref:`apps <snapcraft-app-and-service-metadata>` section exposes the ``bin/os-release.sh`` executable to the host system

Services can be managed with a set of additional commands. See :ref:`Services and daemons <services-and-daemons>` for more information and for further details on other metadata, see :ref:`Snapcraft app and service metadata <snapcraft-app-and-service-metadata>`.

If you need to add user configurable options to your service or daemon, such as which port it should use, see `Adding snap configuration <https://snapcraft.io/docs/adding-snap-configuration>`__.

See `Tab completion <https://snapcraft.io/docs/tab-completion-for-snaps>`__ if you wish to add command line tab completion to your snap.

   ℹ **Interfaces** enable an app to access system resources. Interfaces that are required for normal operation are specified at snap build-time within the above ``app`` metadata of a snap’s snapcraft.yaml. See :ref:`Adding Interfaces <adding-interfaces>` for more details.
