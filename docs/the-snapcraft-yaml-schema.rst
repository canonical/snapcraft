.. 8337.md

.. _the-snapcraft-yaml-schema:

The snapcraft.yaml schema
=========================

The *snapcraft.yaml* file is the main entry point to create a snap through Snapcraft. It contains all the details the *snapcraft* command needs to build a snap.

.. note::

   See :ref:`Snapcraft checklist <snapcraft-checklist>` for an overview of the typical project details needed before creating :file:`snapcraft.yaml`.

In general, snapcraft.yaml can be organised into three principle sections:

**1. top-level metadata**, containing values, such as *name*, typically used by the store:

.. code:: yaml

   name: hello
   base: core18
   version: '2.10'
   summary: GNU Hello, the "hello world" snap
   description: |
     GNU hello prints a friendly greeting.
   grade: stable
   confinement: strict

**2. apps** detailing how apps and services are exposed to the host system:

.. code:: yaml

   apps:
     hello:
       command: bin/hello

**3.** and **parts** to describes how to import, and potentially build, each required part of the snap:

.. code:: yaml

   parts:
     gnu-hello:
       source: http://ftp.gnu.org/gnu/hello/hello-2.10.tar.gz
       plugin: autotools

For further details on the metadata contained within each of the above sections, see one of the following:

1. :ref:`Snapcraft top-level metadata <snapcraft-top-level-metadata>`
2. :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`
3. :ref:`Snapcraft app and service metadata <snapcraft-app-and-service-metadata>`

A set of environment variables is also available during the build process. See :ref:`Environment variables that Snapcraft exposes <environment-variables-that-snapcraft-exposes>` for further details.

Additionally, see :ref:`Snapcraft.yaml reference <snapcraft-yaml-reference>` for a complete overview of the metadata supported by snapcraft.yaml, and :ref:`Snapcraft advanced grammar <snapcraft-advanced-grammar>` to learn how to check for specific conditions for certain YAML keys.

For the technical definition of the format, the schema for snapcraft.yaml can be found in the Snapcraft source tree: `snapcraft/snapcraft.json at master · snapcore/snapcraft <https://github.com/snapcore/snapcraft/blob/master/schema/snapcraft.json>`__.
