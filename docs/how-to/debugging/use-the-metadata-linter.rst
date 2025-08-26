.. _how-to-use-the-metadata-linter:

Use the metadata linter
=======================

The following guidelines describe how to address issues flagged by the metadata linter.

To resolve a missing metadata key, add it to the snap's project file.

The following metadata keys are linted:

- ``title``
- ``contact``
- ``license``
- ``donation``
- ``issues``
- ``source-code``
- ``website``

See the :ref:`reference-snapcraft-yaml-top-level-keys` for more information about these keys.

To ignore a key, add it to the ``lint.ignore.metadata`` key.

See :ref:`how-to-disable-a-linter-ignore-specific-keys-of-metadata-linter` for more information.
