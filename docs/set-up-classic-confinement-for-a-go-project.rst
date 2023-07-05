.. 35436.md

.. _set-up-classic-confinement-for-a-go-project:

Set up classic confinement for a Go project
===========================================

Some snaps need to have access to system resources outside the scope allowed by strict confinement, and are unable to do this via the available interfaces. These snaps are configured to use classic confinement and will `need to be reviewed <https://forum.snapcraft.io/t/process-for-reviewing-classic-confinement-snaps/1460>`__ before publication in the Snap Store.

This guide shows how to enable classic confinement for a snap containing `cgo <https://pkg.go.dev/cmd/cgo>`__ binaries built with the Go plugin. The example project used in this guide can be found in `this repository <https://github.com/snapcraft-docs/golang-classic-example>`__.

Change the confinement to classic
---------------------------------

Starting with an existing snapcraft.yaml file, change the ``confinement`` setting to ``classic``:

.. code:: yaml

   confinement: classic

This will cause the snap to be built in a way that gives it access to system resources.

Use linters to identify problems
--------------------------------

Snapcraft uses `linters <https://snapcraft.io/docs/linters>`__ to check for issues during builds. Linters can only be specified in snaps that use the `core22 base <https://snapcraft.io/docs/base-snap>`__. Warnings are still reported for snaps that use older bases.

Run ``snapcraft`` to pack the snap. This may produce warnings like the following:

::

   Lint warnings:
   - classic: bin/main: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'. (https://snapcraft.io/docs/linters-classic)
   - classic: bin/main: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'. (https://snapcraft.io/docs/linters-classic)

If there are many warnings about libraries you can disable the library linter so that only classic linter warnings are shown. See the `linters <https://snapcraft.io/docs/linters>`__ documentation for details.

Fix linter warnings by setting link parameters
----------------------------------------------

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

Run Snapcraft again to rebuild the snap, consulting the `Classic linter <https://snapcraft.io/docs/linters-classic>`__ documentation to resolve further issues.

See also `this article <https://snapcraft.io/blog/the-new-classic-confinement-in-snaps-even-the-classics-need-a-change>`__ for an overview of the classic linter and a discussion of the issues involved in building snaps for classic confinement.
