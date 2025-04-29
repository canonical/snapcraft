.. _how-to-disable-a-linter:

Disable a linter
================

You can disable a :ref:`linter <reference-linters>` for a snap by listing it in the
``lint.ignore`` key.

For example, to disable both built-in linters, add this to your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library


Ignore specific files
---------------------

To disable a linter for a specific file, you can list it under a linter's entry in the
``lint.ignore`` key. The path is relative to the snap directory tree, and supports
wildcard characters (*).

In the following example, the ``classic`` linter is disabled entirely, and the
``library`` linter won't run for the files in ``usr/lib`` that match the specified
pattern:

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library:
          - usr/lib/**/libfoo.so*
