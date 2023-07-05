.. 34627.md

.. _set-up-classic-confinement-for-a-cmake-project:

Set up classic confinement for a CMake project
==============================================

Some snaps need to have access to system resources outside the scope allowed by strict confinement, and are unable to do this via the available interfaces. These snaps are configured to use classic confinement and will :ref:`need to be reviewed <process-for-reviewing-classic-confinement-snaps>` before publication in the Snap Store.

This guide shows how to enable classic confinement for a snap built with the CMake plugin. The example project used in this guide can be found in `this repository <https://github.com/snapcraft-docs/cmake-classic-example>`__.

Change the confinement to classic
---------------------------------

Starting with an existing snapcraft.yaml file, change the ``confinement`` setting to ``classic``:

.. code:: yaml

   confinement: classic

This will cause the snap to be built in a way that gives it access to system resources.

Use linters to identify problems
--------------------------------

Snapcraft uses :ref:`linters <snapcraft-linters>` to check for issues during builds. Linters can only be specified in snaps that use the :ref:`core22 base <base-snaps>`. Warnings are still reported for snaps that use older bases.

Run Snapcraft to build the snap. This may produce warnings like the following:

::

   Lint warnings:
   - classic: usr/bin/cmake-example: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'. (https://snapcraft.io/docs/linters-classic)
   - classic: usr/bin/cmake-example: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'. (https://snapcraft.io/docs/linters-classic)

If there are many warnings about libraries you can disable the library linter so that only classic linter warnings are shown. See the :ref:`linters <snapcraft-linters>` documentation for details.

Fix linter warnings by patching ELF binaries
--------------------------------------------

The easiest way to handle warnings about the ELF interpreter and rpath is to let Snapcraft automatically patch the binaries using ``patchelf``.

This is enabled by default for ``core20`` classic snaps, and can also be enabled for ``core22`` classic snaps if you are using Snapcraft 7.3 or a version from the edge channel. Pass the ``enable-patchelf`` build attribute to the plugin:

.. code:: yaml

       plugin: cmake
       build-attributes:
        - enable-patchelf

This can be removed when automatic patching is enabled for ``core22`` classic snaps in stable releases.

Rebuild the snap
----------------

Run Snapcraft again to rebuild the snap, consulting the :ref:`Classic linter <classic-linter>` documentation to resolve further issues.

See also `this article <https://snapcraft.io/blog/the-new-classic-confinement-in-snaps-even-the-classics-need-a-change>`__ for an overview of the classic linter and a discussion of the issues involved in building snaps for classic confinement.
