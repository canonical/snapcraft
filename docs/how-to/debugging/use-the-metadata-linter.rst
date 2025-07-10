.. _how-to-use-the-metadata-linter:

Use the metadata linter
=======================

The following guidelines describe how to address issues flagged by the metadata linter.

To resolve a missing metadata field, add the missing field to the snap's
``snapcraft.yaml`` file.

Currently these following metadata fields are linted:

.. kitbash-field:: craft_application.models.project.Project title
.. kitbash-field:: project.Project contact
.. kitbash-field:: project.App common_id
    :prepend-name: apps.<app-name>
.. kitbash-field:: project.Project donation
.. kitbash-field:: project.Project issues
.. kitbash-field:: craft_application.models.project.Project license
.. kitbash-field:: project.Project source_code
.. kitbash-field:: project.Project website

To ignore the metadata field, add the field to the ``lint.ignore.metadata`` key.
