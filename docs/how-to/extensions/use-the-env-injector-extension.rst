.. _how-to-use-the-env-injector-extension:

Use the env-injector extension
==============================

The :ref:`reference-env-injector-extension` lets you expose environment variables within
the snap to the user. These variables are accessible by the snap's apps and can modify
their behavior as they would in a bare host environment.

The user has multiple available methods for setting these environment variables. Your
snap's apps and design should be oriented toward the most optimal method, and its
documentation should cover which variables are available and the correct method of
assigning them.


Set up the env-injector extension
---------------------------------

To add the env-injector extension to an app in your snap:

1. Since env-injector is an `experimental extension
   <https://snapcraft.io/docs/supported-extensions#p-80380-experimental-extensions>`_,
   it's blocked by default. To enable experimental extensions during build, your host
   must set the following environment variable when packing with Snapcraft:

   .. code-block:: bash

       SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1

2. Your snap must enable configurations with a `configure hook
   <https://snapcraft.io/docs/supported-snap-hooks#heading--the-configure-hook>`_. If
   your snap doesn't use this hook yet, it needs at minimum an executable file named
   ``configure``, without an extension, in ``snap/hooks/``. It should contain at
   minimum:

   .. code-block:: bash

       #!/bin/sh

       # Optional validation logic

3. In your snap's project file, the target app's ``extensions`` key must list
   ``env-injector``. For example, if your snap had an app named ``server``, the key
   would declare:

   .. code-block:: yaml
       :caption: snapcraft.yaml

       apps:
         server:
           command: run.sh
           daemon: simple
           extensions: [ env-injector ]

Once set up, the user can set any available environment variables for the snap's apps.


Set an environment variable
---------------------------

When an app in a snap has behavior bound to an environment variable, the user can set it
either through the `snap's configuration
<https://snapcraft.io/docs/configuration-in-snaps>`_ or by reading an environment
(``.env``) file.

Environment variables are applied to apps in one of two ways:

- *Globally*, where the environment variable is passed to all apps that use
  env-injector.
- *Locally*, where the environment variable is passed to a specific app that uses
  env-injector. The app's name is taken from its definition in the snap project file.
  The name according to the extension can be :ref:`overridden with an alias
  <use-the-env-injector-give-app-alias>` to avoid naming conflicts.



As a snap configuration option
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The user can set environment variables one at a time as snap configuration options
with the ``snap set`` command.

To set an environment variable for all apps in a snap, the user can call ``snap set``
and target the snap and its app. The passed environment variable name must be lowercase.
For example, to set ``HTTP_PORT=8080`` for all apps in a snap that use the env-injector,
the user would run:

.. code-block:: bash

    sudo snap set <snap-name> env.http-port=8080

To set a local environment variable and target a specific app, they can call ``snap
set`` and prefix the option name with ``apps.<app-name>``. To target only the server app
in the previous example, the user would run:

.. code-block:: bash

    sudo snap set <snap-name> apps.server.env.http-port=8080

The app's name is taken from the snap's project file.

When running ``snap set``, the user must adjust the environment variable name. For the
complete details on how snap options interpret environment variables, see
:ref:`env-injector-naming-rules`.


With an environment file
~~~~~~~~~~~~~~~~~~~~~~~~

The user can pass environment variables in ``.env`` files to the snap with the ``snap
set`` command.

If a snap is confined, its file system needs access to the file, either by storing the
file in its `writable area <https://snapcraft.io/docs/data-locations>`_ or through a
file interface.

For a simple example, to globally export the contents of an environment file stored in
the local host, the user would run:

.. code-block:: bash

    sudo snap set <my-snap> envfile=/var/snap/my-snap/common/config.env

The environment variables inside ``config.env`` are then exported to all apps that use
the extension.

To export the contents of the same file as local environment variables of the server
app, the user would run:

.. code-block:: bash

    bash sudo snap set <my-snap> apps.server.envfile=/var/snap/my-snap/common/server.env


.. _use-the-env-injector-give-app-alias:

Give an app an alias for the environment
----------------------------------------

The app's name is taken from its definition in the snap's project file. You can override
how the app is referred to in the environment by setting its ``env_alias`` key.

For example, to override an app named ``server`` with ``web-server``, the project file
would declare:

.. code-block:: yaml
    :caption: snapcraft.yaml

    apps:
      server:
        command: run.sh
        daemon: simple
        extensions: [ env-injector ]
        environments:
          env_alias: web-server

Then, the user could set a local environment variable on the app with:

.. code-block:: bash

    sudo snap set <my-name> apps.web-server.env.http-port=8080

Similarly, the user could override the app's local ``.env`` file with:

.. code-block:: bash

    sudo snap set <my-name> apps.web-server.envfile=/var/snap/my-snap/common/server.env
