.. _how-to-use-classic-linter:

Use the classic linter
======================

The classic linter is a :ref:`Snapcraft linter <reference-linters>` that is used to
verify binary file parameters to ensure they are set appropriately for snaps using
:ref:`explanation-classic-confinement`.

The classic linter is only invoked when snap confinement is set to ``classic``, or if
``libc`` is staged.

The classic linter will issue a warning if the ELF binary it is testing either:

- does not have the correct dynamic linker (either from the base or the staged
  ``libc``)
- does not have an ``rpath`` (run-time search path) set to a value needed to load
  shared library dependencies, either from the base or from the snap if the dependency
  is not part of the base.

See :ref:`how-to-disable-a-linter` for details on how to stop this linter from running.


Find issues at build time
-------------------------

To address classic linter issues, the appropriate ``rpath`` can be set during build
time, or existing binaries can be patched to have their rpath changed.


Fix Make warnings with linker flags
-----------------------------------

Setting ``rpath`` at build time requires linker parameters to be used. The linker is
typically invoked indirectly via a compiler driver; with gcc, for example, case
parameters can be passed to the linker using the ``-Wl`` option::

    gcc -o foo foo.o -Wl,-rpath=\$ORIGIN/lib,--disable-new-dtags -Llib -lbar

Fix Go warnings with linker flags
---------------------------------

A similar strategy can be used for packages building a `cgo
<https://pkg.go.dev/cmd/cgo>`_ binary:

.. code-block:: go

    package main
    /*
    #cgo LDFLAGS: -L${SRCDIR}/lib -Wl,-rpath=\$ORIGIN/lib -Wl,--disable-new-dtags -lbar
    #include "bar.h"
    */
    import "C"

    func main() {
        C.bar()
    }

Patch ELF binaries
------------------

A snap payload may also contain pre-built ELF binaries installed from arbitrary sources
(typically from the distribution archive, after installing stage packages).

In these cases, ``rpath`` must be set by modifying the existing binary using a tool
such as ``patchelf``::

    patchelf --force-rpath --set-rpath \$ORIGIN/lib foo

Or, to set the ELF interpreter, the following command can be used:

.. code-block:: text

    patchelf --set-interpreter /snap/core22/current/lib64/ld-linux-x86-64.so.2 foo

Enable automatic ELF file patching
----------------------------------

Snapcraft 7.3 is required to perform automatic ELF patching for core22 and up classic
snaps. To use it, declare:

.. code-block:: yaml

    build-attributes:
      - enable-patchelf

in all parts that should have ELF binaries automatically patched.
