.. 32229.md

.. _library-linter:

Library linter
==============

The *library* linter is a :ref:`Snapcraft linter <snapcraft-linters>` that is used to verify whether any ELF file dependencies, typically libraries, are missing from a snap. It also verifies if any libraries are included in a snap but are not used.

See :ref:`Disabling linters <snapcraft-linters-disable>` for details on how to stop this linter running.


.. _library-linter-help:

How the linter helps
--------------------

If a snapped application depends on libraries neither provided by the :ref:`base <base-snaps>` system, nor included as a dependency, the snap package may execute in an incorrect or unpredictable way. The library linter issues a warning if a missing dependency is detected.

Snap package size can be reduced by removing extra libraries that are not used by the applications in the snap. The library linter issues a warning if unused libraries are detected.


.. _library-linter-warnings:

Linter warnings
---------------

The library linter will issue a warning if:

-  ELF dependencies are detected as missing from the snap package.
-  Libraries not used by an ELF file in the snap package are detected.


.. _library-linter-warnings-example:

Example
~~~~~~~

The example below shows two missing libraries (``libcaca`` and ``libslang``) and one unused library (``libpng16``).

.. code:: text

   Running linter: library
   Lint warnings:
   - library: my-app: missing dependency 'libcaca.so.0'.
   - library: my-app: missing dependency 'libslang.so.2'.
   - library: libpng16.so.16: unused library 'usr/lib/x86_64-linux-gnu/libpng16.so.16.37.0'.


.. _library-linter-issues:

Addressing issues
-----------------

To address library linter issues, packages containing any missing libraries need to be added to the list of ``stage-packages``. Unused libraries can be removed from ``stage-packages``. See :ref:`Build and staging dependencies <build-and-staging-dependencies>` for further details.
