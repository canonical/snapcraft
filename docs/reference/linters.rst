Linters
=======

A *linter* is an analysis tool that checks for common errors or compatibility
issues, usually automatically, or as part of some other process.

Snapcraft (from version 7.2 onwards) includes its own linter functionality when
working with snaps using a ``core22`` or newer :doc:`base</reference/bases>`.

Snapcraft linters run automatically when a snap is packed unless otherwise
disabled.

Available linters
-----------------

Snapcraft runs the following linters:

- `classic`_: verifies binary file parameters for snaps using classic
  `confinement`_

- `library`_: verifies that no ELF file dependencies, such as libraries, are
  missing and that no extra libraries are included in the snap package

Disabling linters
~~~~~~~~~~~~~~~~~

Snapcraft linters run automatically when a snap is packed but specific linters
can be disabled by specifying a ``ignore`` entry in the ``lint`` section of
``snapcraft.yaml``:

.. code-block:: yaml

   lint:
     ignore:
       - classic
       - library

The ``ignore`` entry must include one or more [linter names](#heading--linters)
for those linters to be disabled.

Ignore specific files
~~~~~~~~~~~~~~~~~~~~~

To omit specific files from a linter, add their snap location under the linter
name:

.. code-block:: yaml

   lint:
     ignore:
       - classic
       - library:
         - usr/lib/**/libfoo.so*

In the above example, the ``classic`` linter will be disabled entirely, and the
``library`` linter will not run for the files matching the specified file
pattern.


.. _classic: https://snapcraft.io/docs/linters-classic
.. _confinement: https://snapcraft.io/docs/snap-confinement
.. _library: https://snapcraft.io/docs/linters-library
