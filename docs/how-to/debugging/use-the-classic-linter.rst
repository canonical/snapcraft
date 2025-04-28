.. _how-to-use-the-classic-linter:

Use the classic linter
======================

The *classic linter* in Snapcraft verifies binary file parameters to ensure they are set
appropriately for snaps. The classic linter is only invoked when a snap has
:ref:`classic confinement <explanation-classic-confinement>` or it stages ``libc``.

The classic linter will issue a warning if the ELF binary it is testing either:

- Doesn't have the correct dynamic linker (either from the base or the staged
  ``libc``), or
- Doesn't have an ``rpath`` (run-time search path) set to a value needed to load
  shared library dependencies, either from the base or from the snap if the dependency
  is not part of the base.

See :ref:`how-to-disable-a-linter` for details on how to stop this linter from running.


Fix Make warnings with linker flags
-----------------------------------

In this example, the warnings about the ELF interpreter and ``rpath`` can be handled by
adding options to the linker:

- ``-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2``
- ``-Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu``

In an :ref:`Autotools <craft_parts_autotools_plugin>` project, if the ``LDFLAGS``
environment variable is used, the project file can be updated to pass these options to
the Autotools plugin. This is done by using the ``autotools-configure-parameters`` key:

.. code-block:: yaml

    plugin: autotools
    source: .
    autotools-configure-parameters:
      - LDFLAGS="-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2
                 -Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu"

In a Makefile-based project, if the ``LDFLAGS`` environment variable is used, the
project file can be updated to pass these options to the Make plugin, like this:

.. code-block:: yaml

    plugin: make
    source: .
    make-parameters:
      - LDFLAGS="-Wl,-dynamic-linker=/snap/core22/current/lib64/ld-linux-x86-64.so.2
                 -Wl,-rpath=/snap/core22/current/lib/x86_64-linux-gnu"

Both of these options will only be useful for projects where the ``LDFLAGS`` variable
can be used to influence the build process.

Fix Go warnings with linker flags
---------------------------------

Unlike regular binaries built with gcc, Go ELF binaries may not be correctly patched
using patchelf. In this case, it's necessary to add the appropriate parameters to the
linker so that the executable is created with the correct ELF interpreter and
``rpath``. To do so, add the following ``#cgo`` directive to your source code:

.. code-block:: go

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

You may also need to define the following environment variables in the part's
``build-environment`` key:

.. code-block:: yaml

    plugin: go
    # ...
    build-environment:
      - CGO_ENABLED: 1
      - CGO_LDFLAGS_ALLOW: ".*"

For more information on ``cgo``, see the `Go docs <https://pkg.go.dev/cmd/cgo>`_.

Patch ELF binaries
------------------

If the other methods aren't applicable to the build process used by a snap, the
``rpath`` can instead be set by modifying the existing binary using a tool such as
``patchelf``::

    patchelf --force-rpath --set-rpath \$ORIGIN/lib foo

Or, to set the ELF interpreter, the following command can be used:

.. code-block:: text

    patchelf --set-interpreter /snap/core22/current/lib64/ld-linux-x86-64.so.2 foo

This can be done using override scripts in order to patch the binaries as part of
the packaging of the snap. For more information, see `Override build steps
<https://snapcraft.io/docs/overrides>`_.

Enable automatic ELF file patching
----------------------------------

Snapcraft 7.3 or higher is required to perform automatic ELF patching for core22 and up
classic snaps. To use it, declare:

.. code-block:: yaml

    build-attributes:
      - enable-patchelf

in all parts that should have ELF binaries automatically patched.
