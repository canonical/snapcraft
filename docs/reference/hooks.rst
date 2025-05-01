.. _reference-hooks:

Hooks
=====

A hook is an executable file that runs within a snap’s confined environment when
a certain action occurs.

Common scenarios requiring hooks include:

- Notifying a snap that something has happened. For example, if a snap has been
  upgraded, the snap may need to trigger a scripted migration process to port an
  old data format to the new one.

- Notifying a snap that a specific operation is in progress. For example, a snap may
  need to know when a specific interface connects or disconnects.

Transactions and rollback
-------------------------

Snapd executes a hook as a single transaction that holds all the configuration changes
made by the hook. These changes are atomic and are only visible to the running system
after the hook finishes executing.

This allows changes to be rolled back if errors occur during the execution of a hook.
This happens if a non-zero value is returned with either the configure or
default-configure hooks or if an error occurs with any hook involved with
an `interface auto-connection <https://snapcraft.io/docs/auto-connection-mechanism>`_.

Environment
-----------

If the hook executable is a shell script, it should assume a POSIX-compliant shell
environment for its execution.

If your script needs a specific interpreter, such as Bash or Python, it needs to be
explicitly declared within the script’s shebang header (``#!/bin/bash``, for example)
and your snap needs to ensure the interpreter is available.

Hook names
----------

The filename of the executable should be the name of the hook. If the file
exists, snapd will execute the file when required by that hook’s action. See
`Supported snap hooks <https://snapcraft.io/docs/supported-snap-hooks>`_
for details on supported hooks.

Adding hooks to a snap
----------------------

A hook executable can be added to a snap by either placing it in the project's
``snap/hooks/`` directory or by generating it at build time with a part.

Adding a hook from the project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A hook can be added to a snap by placing the hook executable in the project's
``snap/hooks/`` directory. The following example shows the location of a configure
hook executable:

.. terminal::

   .
   └── snap
       ├── hooks
       │   └── configure
       └── snapcraft.yaml

Hook executables in ``snap/hooks/`` are automatically copied into the snap during
the prime step. See :ref:`explanation-parts-lifecycle` for details.

Adding a hook from a part
~~~~~~~~~~~~~~~~~~~~~~~~~

A part can add a hook to a snap by installing the hook executable into
``$CRAFT_PART_INSTALL/snap/hooks/`` at build time. The following example shows a
part that adds a configure hook using the :ref:`craft_parts_dump_plugin` and the
``organize`` key.

.. code-block:: yaml
    :caption: snapcraft.yaml

    configure-hook:
      plugin: dump
      source: src/hooks
      organize:
        configure: snap/hooks/configure

Accessing resources
-------------------

If a hook requires access to system resources outside of a snap’s confined environment,
it needs to use `interfaces <https://snapcraft.io/docs/interface-management>`_ to
access those resources.

The following example registers an install hook making use of the `network
<https://snapcraft.io/docs/network-interface>`_ interface:

.. code-block:: yaml
    :caption: snapcraft.yaml

    hooks:
      install:
        plugs: [network]

Snapd executes hooks with no parameters. If a hook needs to request or modify
information, they can use the snapctl tool. See `Using the snapctl tool
<https://snapcraft.io/docs/using-snapctl>`_ for details.
