.. _how-to-use-the-metadata-linter:

Use the metadata linter
=======================

The following guidelines describe how to address issues flagged by the metadata linter.

To resolve a missing metadata field, add the missing field to the snap's
``snapcraft.yaml`` file.

Currently these following metadata fields are linted:

- ``title``
- ``contact``
- ``license``
- ``donation``
- ``issues``
- ``source-code``
- ``website``

See the :ref:`reference-snapcraft-yaml-top-level-keys` for more information about these keys.

To ignore the metadata field, add the field to the ``lint.ignore.metadata`` key. See :ref:`how-to-disable-a-linter-ignore-specific-keys-of-metadata-linter` for more information.
