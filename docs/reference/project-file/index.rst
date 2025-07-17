.. _reference-project-file:

Project file
============

Every snap project depends on a file called :ref:`reference-snapcraft-yaml` which
instructs Snapcraft how to build the snap.

This project file is written in `YAML <https://yaml.org/>`_ and can be located in
any of the following locations in the root directory of the project where Snapcraft
is run:

- ``snapcraft.yaml``
- ``snap/snapcraft.yaml``
- ``.snapcraft.yaml``
- ``build-aux/snap/snapcraft.yaml``

The project file is organized into three main sections:

- Top-level directives, which provide information about the snap and describe its base
  properties and publication details.
- App directives, which describe the execution, interfaces, and resources available
  to each app in the snap.
- Part directives, which describe how to import and build the apps inside the snap.

You can create a project file manually or by running the ``init`` command, which
creates a basic template:

.. code-block:: bash

    snapcraft init

For a complete reference of the keys in a project file, see
:ref:`reference-snapcraft-yaml`.

For a description of the project file's structure, see
:ref:`reference-anatomy-of-snapcraft-yaml`.

.. toctree::
    :hidden:

    snapcraft-yaml
    anatomy-of-snapcraft-yaml
