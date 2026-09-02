.. _how-to-debug-classic-confinement:

Debug classic confinement
=========================

This guide provides solutions to issues related to ``RPATH`` and ELF binaries in classic
snaps.


Setting RPATH from sources with Snapcraft
-----------------------------------------

An ELF binary created during the parts lifecycle execution can have its ``RPATH`` value
set by using appropriate linker parameters. The linker is typically invoked indirectly
by a compiler driver. In the gcc case, parameters can be passed to the linker using the
``-Wl`` option:

.. code-block:: bash

    gcc -o foo foo.o -Wl,-rpath=\$ORIGIN/lib,--disable-new-dtags -Llib -lbar


Setting ``RPATH`` for pre-built binaries
----------------------------------------

Snaps may contain pre-built ELF binaries installed from arbitrary sources, typically
from the distribution repository archives, after installing stage packages.

In such a case, you must set ``RPATH`` by modifying the existing binary with a tool such
as `PatchELF <https://snapcraft.io/patchelf>`_:

.. code-block:: bash

    patchelf --force-rpath --set-rpath \$ORIGIN/lib “binary file”

You can also use PatchELF to change the interpreter to a different dynamic linker:

.. code-block:: bash

    patchelf --set-interpreter /lib64/ld-linux-x86-64.so.2 foo


Possible conflicts
------------------

Patching ELF binaries to modify ``RPATH`` or interpreter entries may fail in certain cases, as with binaries using libc variants that require a nonstandard interpreter. Additionally, patching will cause signed binaries to change the signature of the binaries, which may have the side effect of failed validation for tools or scenarios where the software hashes were generated beforehand.
