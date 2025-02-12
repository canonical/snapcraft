Linters
=======

A *linter* is an analysis tool that checks for common errors or compatibility
issues, usually automatically, or as part of some other process.

Snapcraft 7.2 and higher provides built-in linter functionality when the snap
uses core22 or higher as its :doc:`base </reference/bases>`.

By default, these built-in linters run automatically when a snap is built. If
they're unneeded, you can disable them in the snap's project file.

Built-in linters
-----------------

Snapcraft runs the following linters:

- `classic`_. Verifies binary file parameters for snaps using
  `classic confinement`_.

- `library`_. Verifies that no ELF file dependencies, such as libraries, are
  missing, and that no extra libraries are included in the snap package.

Disable a linter
----------------

You can disable a linter by adding it to the ``lint.ignore`` key in
the project file. For example:

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library


Ignore specific files
~~~~~~~~~~~~~~~~~~~~~

To disable a linter for a specific file, you can list it under a linter's entry
in the ``lint.ignore`` key. The path is relative to the snap directory tree,
and supports wildcard characters (**\***).

In the following example, the ``classic`` linter is disabled entirely, and the
``library`` linter won't run for the files in ``usr/lib`` that match the
specified pattern:

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library:
          - usr/lib/**/libfoo.so*


.. _classic: https://snapcraft.io/docs/linters-classic
.. _classic confinement: https://snapcraft.io/docs/snap-confinement
.. _library: https://snapcraft.io/docs/linters-library
