.. 34179.md

.. _set-up-classic-confinement-for-a-python-project:

Set up classic confinement for a Python project
===============================================

Some snaps need to have access to system resources outside the scope allowed by strict confinement, and they are unable to do this via the available interfaces. These snaps are configured to use classic confinement and will :ref:`need to be reviewed <process-for-reviewing-classic-confinement-snaps>` before publication in the Snap Store.

This guide shows how to enable classic confinement for a snap built with the :ref:`python plugin <the-python-plugin>`. The example project used in this guide can be found in the `example repository <https://github.com/snapcraft-docs/python-ctypes-example>`__.

Change the confinement to classic
---------------------------------

Starting with an existing snapcraft.yaml file, change the ``confinement`` setting to ``classic``:

.. code:: yaml

   confinement: classic

This will cause the snap to be built in a way that gives it access to system resources.

Patch ctypes to load system libraries
-------------------------------------

If your application uses `ctypes <https://docs.python.org/3/library/ctypes.html>`__ to access system libraries it will need to be bundled with a patched version of the module. To bundle ``ctypes``, include the relevant packages in the ``stage-packages`` list of packages. For the :ref:`core22 base <base-snaps>`, the packages will be the following:

.. code:: yaml

       stage-packages:
         - libpython3.10-minimal
         - libpython3.10-stdlib

Patching is done by overriding the build step to perform the build as normal, by running ``snapcraftctl build``, then applying `a patch <https://github.com/snapcraft-docs/python-ctypes-example/blob/main/snap/local/patches/ctypes_init.diff>`__ to the staged module file with a script:

.. code:: yaml

       override-build: |
         snapcraftctl build
         $SNAPCRAFT_PROJECT_DIR/snap/local/patch-ctypes.sh

An `example script and patch <https://github.com/snapcraft-docs/python-ctypes-example/tree/main/snap/local>`__ can be found in the `example repository <https://github.com/snapcraft-docs/python-ctypes-example>`__.

Run Snapcraft to build the snap with these changes. This may produce warnings about run-time library paths like the following:

::

   Lint warnings:
   - classic: usr/lib/python3.10/lib-dynload/_asyncio.cpython-310-x86_64-linux-gnu.so: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'. (https://snapcraft.io/docs/linters-classic)
   - classic: usr/lib/python3.10/lib-dynload/_bz2.cpython-310-x86_64-linux-gnu.so: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'. (https://snapcraft.io/docs/linters-classic)

These warnings tell us that the run-time library paths of some core Python modules need to be patched to refer to libraries in the base snap.

Fix linter warnings by patching ELF binaries
--------------------------------------------

The easiest way to handle warnings about the ELF interpreter and rpath is to let Snapcraft automatically patch the binaries using ``patchelf``.

This is enabled by default for ``core20`` classic snaps, and can also be enabled for ``core22`` classic snaps if you are using Snapcraft 7.3 or a version from the edge channel. Pass the ``enable-patchelf`` build attribute to the plugin:

.. code:: yaml

       build-attributes:
        - enable-patchelf

This can be removed when automatic patching is enabled for ``core22`` classic snaps in stable releases.

Rebuild the snap
----------------

Run Snapcraft again to rebuild the snap, consulting the :ref:`Classic linter <classic-linter>` documentation to resolve further issues.

See also `this article <https://snapcraft.io/blog/the-new-classic-confinement-in-snaps-even-the-classics-need-a-change>`__ for an overview of the classic linter and a discussion of the issues involved in building snaps for classic confinement.
