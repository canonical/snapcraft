.. 32211.md

.. _snapcraft-linters:

Snapcraft linters
=================

A *linter* is an analysis tool that checks for common errors or compatibility issues, usually automatically, or as part of some other process.

Snapcraft (from version 7.2 onwards) includes its own linter functionality when working with snaps using the ``core22`` :ref:`base <base-snaps>`.

Snapcraft linters run automatically when a snap is packed unless otherwise :ref:`disabled <snapcraft-linters-disable>`.


.. _snapcraft-linters-linters:

Available linters
-----------------

Snapcraft currently offers the following linters:

-  :ref:`classic <classic-linter>`: verifies binary file parameters for snaps using :ref:`classic confinement <snap-confinement>`
-  :ref:`library <library-linter>`: verifies that no ELF file dependencies, such as libraries, are missing and that no extra libraries are included in the snap package


.. _snapcraft-linters-disable:

Disabling linters
-----------------

Snapcraft linters run automatically when a snap is packed but specific linters can be disabled by specifying a ``ignore`` entry in the ``lint`` section of ``snapcraft.yaml``:

.. code:: yaml

   lint:
     ignore:
       - classic
       - library

The ``ignore`` entry must include one or more `linter names <snapcraft-linters-linters_>`__ for those linters to be disabled.


.. _snapcraft-linters-disable-files:

Ignore specific files
~~~~~~~~~~~~~~~~~~~~~

To omit specific files from a linter, add their snap location under the linter name:

.. code:: yaml

   lint:
     ignore:
       - classic
       - library:
         - usr/lib/**/libfoo.so*

In the above example, the ``classic`` linter will be disabled entirely, and the ``library`` linter will not run for the files matching the specified file pattern.


.. toctree::
   :hidden:

   classic-linter
   library-linter
