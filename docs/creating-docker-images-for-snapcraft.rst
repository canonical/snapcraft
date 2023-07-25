.. 11739.md

.. _creating-docker-images-for-snapcraft:

Creating docker images for snapcraft
====================================

:ref:`Snapcraft <snapcraft-overview>` is delivered as a snap and this creates a few complications when used with Docker. In particular, incompatible design constraints imposed by Docker affect the way snaps are intended to run. Fortunately, there are workarounds that can be used to create Docker images that will build snaps using Snapcraft.

What follows is a ``Dockerfile`` that can creates a docker image for Snapcraft from the stable channel: https://raw.githubusercontent.com/snapcore/snapcraft/master/docker/Dockerfile

The above ``Dockerfile`` can be used to build snaps with a :file:`snapcraft.yaml` file that either 1) defines ``base: core``, or 2) doesn’t define a base at all. In both cases, the build environment is expected to be Ubuntu 16.04 LTS (Xenial Xerus).

To create a ``Dockerfile`` that can be used with ``base: core18`` in :file:`snapcraft.yaml` (the current default), change the line ``FROM ubuntu:xenial`` to ``FROM ubuntu:bionic`` in `Dockerfile <https://raw.githubusercontent.com/snapcore/snapcraft/master/docker/Dockerfile>`__:

For more information on bases, see :ref:`Base snaps <base-snaps>`.

   ℹ Reduce a snap’s build time by extending your project-specific Docker image to pre-install most of a snap’s dependencies, as defined by its :file:`snapcraft.yaml` file.
