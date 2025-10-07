.. _how-to-disable-a-linter:

Disable a linter
================

You can disable a :ref:`linter <reference-linters>` for a snap by listing it in the
``lint.ignore`` key.

For example, to disable all built-in linters, add this to your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library
        - metadata


.. _how-to-disable-a-linter-ignore-specific-files:

Ignore files and keys
---------------------

To disable a linter for a specific file, you can list it under a linter's entry in the
``lint.ignore`` key. The path is relative to the snap directory tree, and supports
wildcard characters (*).

In the following example, the ``classic`` linter is disabled entirely, the
``library`` linter won't run for the files in ``usr/lib`` that match the specified
pattern.

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - classic
        - library:
          - usr/lib/**/libfoo.so*


.. _how-to-disable-a-linter-ignore-specific-keys-of-metadata-linter:

Ignore specific keys of the metadata linter
-------------------------------------------

You can disable the metadata linter for specific keys by listing them in the
``lint.ignore`` key. The keys are from the top level keys of a project file.

In the following example, the ``title`` and ``issues`` keys are ignored by the
metadata linter.

.. code-block:: yaml
    :caption: snapcraft.yaml

    lint:
      ignore:
        - metadata:
          - title
          - issues
