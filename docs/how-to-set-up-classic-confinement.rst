.. 34416.md

.. _how-to-set-up-classic-confinement:

How to set up classic confinement
=================================

Some snaps need to have access to system resources outside the scope allowed by strict confinement, and are unable to do this via the available interfaces. These snaps are configured to use classic confinement and will :ref:`need to be reviewed <process-for-reviewing-classic-confinement-snaps>` before publication in the Snap Store.

This guide shows how to enable classic confinement for a snap built with a plugin. The example project used in this guide can be found in the appropriate repository in the list below:

-  :ref:`autotools plugin <the-autotools-plugin>`: `example <https://github.com/snapcraft-doc-samples-unofficial/autotools-classic-example>`__
-  :ref:`cmake plugin <the-cmake-plugin>`: `example <https://github.com/snapcraft-docs/cmake-classic-example>`__
-  :ref:`go plugin <the-go-plugin>`: `example <https://github.com/snapcraft-docs/golang-classic-example>`__

   -  when using `cgo <https://pkg.go.dev/cmd/cgo>`__ binaries

-  :ref:`make plugin <the-make-plugin>`: `example <https://github.com/snapcraft-doc-samples-unofficial/makefile-lib-example>`__
-  :ref:`python plugin <the-python-plugin>`: `example <https://github.com/snapcraft-docs/python-ctypes-example>`__

Change the confinement to classic
---------------------------------

Starting with an existing snapcraft.yaml file, change the ``confinement`` setting to ``classic``:

.. code:: yaml

   confinement: classic

This will cause the snap to be built in a way that gives it access to system resources.

Python: patch ctypes to load system libraries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

We need to run Snapcraft to build the snap with these changes to look for core Python modules that will need to be patched to refer to libraries in the base snap.

Use linters to identify problems
--------------------------------

Snapcraft uses :ref:`linters <snapcraft-linters>` to check for issues during builds. Linters can only be specified in snaps that use the :ref:`core22 base <base-snaps>`. Warnings are still reported for snaps that use older bases.

Run Snapcraft to build the snap. This may produce warnings like the following:

::

   Lint warnings:
    - classic: usr/bin/classic-example: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
    - classic: usr/bin/classic-example: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'.

If there are many warnings about libraries you can disable the library linter so that only classic linter warnings are shown. See the :ref:`linters <snapcraft-linters>` documentation for details.

Fix linter warnings by patching ELF binaries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The easiest way to handle warnings about the ELF interpreter and rpath is to let Snapcraft automatically patch the binaries using ``patchelf``.

This is enabled by default for ``core20`` classic snaps, and can also be enabled for ``core22`` classic snaps if you are using Snapcraft 7.3 or a version from the edge channel. Pass the ``enable-patchelf`` build attribute to the ``plugin`` section of the part description:

.. code:: yaml

       build-attributes:
        - enable-patchelf

This can be removed when automatic patching is enabled for ``core22`` classic snaps in stable releases.

autotools/Makefile: fix linter warnings with linker flags
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this example, the warnings about the ELF interpreter and rpath can be handled by adding options to the linker:

-  ``-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2``
-  ``-Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu``

In an autotools project, if the ``LDFLAGS`` environment variable is used, the :file:`snapcraft.yaml` file can be updated to pass these options to the ``autotools`` plugin, using the ``autotools-configure-parameters`` keyword for projects using the ``core20`` base or later:

.. code:: yaml

       plugin: autotools
       source: .
       autotools-configure-parameters:
        - LDFLAGS="-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2
                   -Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu"

This will only be useful for projects where the ``LDFLAGS`` variable can be used to influence the build process.

In a Makefile project, if the ``LDFLAGS`` environment variable is used, the :file:`snapcraft.yaml` file can be updated to pass these options to the ``make`` plugin, like this:

.. code:: yaml

       plugin: make
       make-parameters:
         - LDFLAGS="-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2 -Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu"

This will only be useful for projects where the ``LDFLAGS`` variable can be used to influence the build process.

go: fix linter warnings by setting link parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Unlike regular binaries built with gcc, Go ELF binaries may not be correctly patched using ``patchelf``. In this case, it’s necessary to add the appropriate parameters to the linker so that the executable is created with the correct ELF interpreter and rpath. To do so, add the following ``#cgo`` parameter to your source code:

.. code:: go

   /*
   #cgo LDFLAGS: -L${SRCDIR}/lib -Wl,-rpath=\$ORIGIN/lib:/snap/core22/current/lib/x86_64-linux-gnu -Wl,--disable-new-dtags -Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2 -lzstd
   #include <zstd.h>
   */

   func zstdVersion() int {
           return int(C.ZSTD_versionNumber())
   }

   func main() {
           fmt.Println("libzstd version is", zstdVersion())
   }

You may also need to define the following environment variables in the part’s ``build-environment``:

.. code:: yaml

   parts:
     golang-classic-example:
       plugin: go
       ...
       build-environment:
         - CGO_ENABLED: 1
         - CGO_LDFLAGS_ALLOW: ".*"

Rebuild the snap
----------------

Run Snapcraft again to rebuild the snap, consulting the :ref:`Classic linter <classic-linter>` documentation to resolve further issues.

See also `this article <https://snapcraft.io/blog/the-new-classic-confinement-in-snaps-even-the-classics-need-a-change>`__ for an overview of the classic linter and a discussion of the issues involved in building snaps for classic confinement.
