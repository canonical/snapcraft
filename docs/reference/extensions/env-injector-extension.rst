.. _reference-env-injector-extension:

env-injector extension
======================

The env-injector extension, referred to internally as ``env-injector``, provides an
interface for altering snap behavior with user-defined environment variables.

When you add the extension to individual apps inside a snap, you open the way for the
user to `configure the snap <https://snapcraft.io/docs/configuration-in-snaps>`_ or
affect its behaviour with environment variables.


How it works
------------

This extension splits environment variables into two types:

- *Global* environment variables are visible to all apps in the snap that use
  env-injector.
- *Local* environment variables are visible only to apps with env-injector that the user
  specifically targets.

At a high level, the extension has the following processes and rules during build and
runtime.


Environment variable order of precedence
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Environment variables are processed in the following order:

1. Variables extracted from the global ``.env`` file.
2. Variables extracted from the local ``.env`` file.
3. Variables from global snap config.
4. Variables from local snap config.


App build process
~~~~~~~~~~~~~~~~~

.. TODO: Put a link to exporter program repository

For each app that uses the extension:

1. The app responsible for processing environment variables (the exporter program) is
   added to its `command chain
   <https://snapcraft.io/docs/snapcraft-yaml-schema#p-21225-command-chain-47>`_.
2. The app's name is taken either from its internal name or an :ref:`environment alias
   <use-the-env-injector-give-app-alias>`.


Runtime process
~~~~~~~~~~~~~~~

When the snap runs, for each app that uses the extension, the exporter program:

1. Runs before the app is executed.
2. Makes a request to the `snapd API <https://snapcraft.io/docs/using-the-api>`_
   through the snapd Unix socket.
3. Reads the available environment variables or paths to the ``.env`` files.
4. Translates the snap options into environment variables.
5. If available, loads the ``.env`` files and sets the listed variables.
6. Sets all converted environment variables.
7. Executes the app's command.


.. _env-injector-naming-rules:

Snap option naming and rules
----------------------------

Because there's a chance of conflict and collision between the names of snap options and
environment variables, this extension is governed by naming rules.

Snap options map to environment variables:

.. list-table::
    :header-rows: 1

    * - Type
      - Snap configuration option
      - Environment variable
    * - Global
      - ``env.<key>=<value>``
      - ``<key>=<value>``
    * - Local
      - ``apps.<app>.env.<key>=<value>``
      - ``<key>=<value>``

When mapped, key names are sanitized before being converted into environment variable
names:

.. list-table::
    :header-rows: 1

    * - Character in snap option
      - Result in environment variable
    * - Lowercase letters
      - Uppercase letters
    * - Number not at key start
      - Number
    * - Hyphen not at key start or end
      - Underscore

.. tip::

    The first column is the rule set for snap option names. Any character not detailed
    in it is invalid in a snap option name.


User-selected environment files
-------------------------------

With the special ``envfile`` snap options, the user can also pass whole environment
(``.env``) files to the snap, provided the file is already packed inside the snap.

.. list-table::
    :header-rows: 1

    * - Type
      - Snap option
    * - Global
      - ``envfile=<path>``
    * - Local
      - ``apps.<app>.envfile=<path>``
