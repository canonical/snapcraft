.. 32228.md

.. _classic-linter:

Classic linter
==============

The *classic* linter is a :ref:`Snapcraft linter <snapcraft-linters>` that is used to verify binary file parameters to ensure they are set appropriately for snaps using :ref:`classic confinement <snap-confinement>`.

The classic linter is only invoked when snap confinement is set to ``classic``, or if *libc* is staged.

See :ref:`Disabling linters <snapcraft-linters-disable>` for details on how to stop this linter running.


.. _classic-linter-help:

How the linter helps
--------------------

Unlike un-snapped applications, snaps using classic confinement require dynamic executables to load shared libraries from the appropriate :ref:`base snap <base-snaps>` rather from than the host’s root filesystem.

To prevent version and platform incompatibly issues, snap-based binaries need to be either built with appropriate linker parameters, or patched to allow loading shared libraries from their base snap. The classic linter helps with this by warning about libraries that need to be patched.


.. _classic-linter-warnings:

Linter warnings
---------------

The classic linter will issue a warning if the ELF binary it is testing:

1. does not use the correct dynamic linker (either from the ``base`` or the staged *libc*).
2. does not have an *rpath* (run-time search path) set to a value needed to load shared library dependencies, either from the ``base`` or from the snap if the dependency is not part of the ``base``.

.. code:: bash

   Running linter: classic
   Lint OK:
   - classic: Snap confinement is set to classic.
   Lint warnings:
   - classic: usr/bin/toilet: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/bin/toilet: ELF rpath should be set to '$ORIGIN/../lib/x86_64-linux-gnu:/snap/core22/current/lib/x86_64-linux-gnu'.
   - classic: usr/lib/x86_64-linux-gnu/caca/libgl_plugin.so.0.0.0: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/lib/x86_64-linux-gnu/caca/libgl_plugin.so.0.0.0: ELF rpath should be set to '$ORIGIN/..:/snap/core22/current/lib/x86_64-linux-gnu'.
   - classic: usr/lib/x86_64-linux-gnu/caca/libx11_plugin.so.0.0.0: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/lib/x86_64-linux-gnu/caca/libx11_plugin.so.0.0.0: ELF rpath should be set to '$ORIGIN/..:/snap/core22/current/lib/x86_64-linux-gnu'.
   - classic: usr/lib/x86_64-linux-gnu/libcaca++.so.0.99.19: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/lib/x86_64-linux-gnu/libcaca++.so.0.99.19: ELF rpath should be set to '$ORIGIN:/snap/core22/current/lib/x86_64-linux-gnu'.
   - classic: usr/lib/x86_64-linux-gnu/libcaca.so.0.99.19: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/lib/x86_64-linux-gnu/libcaca.so.0.99.19: ELF rpath should be set to '$ORIGIN:/snap/core22/current/lib/x86_64-linux-gnu'.
   - classic: usr/lib/x86_64-linux-gnu/libslang.so.2.3.2: ELF interpreter should be set to '/snap/core22/current/lib64/ld-linux-x86-64.so.2'.
   - classic: usr/lib/x86_64-linux-gnu/libslang.so.2.3.2: ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'.




.. _classic-linter-issues:

Addressing issues
-----------------

To address classic linter issues, the appropriate *rpath* can be set during build time, or existing binaries can be patched to have their rpath changed.


.. _classic-linter-issues-build:

At build time
~~~~~~~~~~~~~

Setting *rpath* at build time requires linker parameters to be used. The linker is typically invoked indirectly via a compiler driver; with *gcc*, for example, case parameters can be passed to the linker using the ``-Wl`` option:

.. code:: bash

   gcc -o foo foo.o -Wl,-rpath=\$ORIGIN/lib,--disable-new-dtags -Llib -lbar

A similar strategy can be used to set rpath in a `cgo <https://pkg.go.dev/cmd/cgo>`__ binary:

.. code:: go

   package main
   /*
   #cgo LDFLAGS: -L${SRCDIR}/lib -Wl,-rpath=\$ORIGIN/lib -Wl,--disable-new-dtags -lbar
   #include "bar.h"
   */
   import "C"

   func main() {
       C.bar()
   }

Linker argument ``-Wl,-dynamic-linker=...`` can be used to set the ELF interpreter.


.. _classic-linter-issues-binary:

Binary patching
~~~~~~~~~~~~~~~

A snap payload may also contain pre-built ELF binaries installed from arbitrary sources (typically from the distribution archive, after installing stage packages).

In these cases, rpath must be set by modifying the existing binary using a tool such as `patchelf <https://manpages.ubuntu.com/manpages/xenial/man1/patchelf.1.html>`__:

.. code:: bash

   patchelf --force-rpath --set-rpath \$ORIGIN/lib foo

Or, to set the ELF interpreter, the following command can be used:

.. code:: bash

   patchelf --set-interpreter /snap/core22/current/lib64/ld-linux-x86-64.so.2 foo


.. _classic-linter-issues-auto:

Automatic ELF file patching
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 7.2 does not currently perform automatic ELF patching for ``base: core22`` classic snaps, however this feature is now available in edge. To use it, declare:

.. code:: yaml

    build-attributes:
     - enable-patchelf

in all parts that should have ELF binaries automatically patched.

If automatic ELF file patching is required in a stable channel, use ``base: core20`` until Snapcraft 7.3 is released to stable.
