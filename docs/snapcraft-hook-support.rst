.. 19069.md

.. _snapcraft-hook-support:

Snapcraft hook support
======================

A hook is an executable file that runs within a snap’s confined environment when a specific action occurs.

The filename of the executable is based on the name of the hook. If this file exists, *snapd* will execute the file when required by that hook’s action. See :ref:`Supported snap hooks <supported-snap-hooks>` for more details on which hooks are supported.

:ref:`Snapcraft <snapcraft-overview>` can integrate hooks into a snap using two methods:

1. Using a project-wide ``snap/hooks`` directory
------------------------------------------------

The hook executable can be placed in a directory called ``snap/hooks`` relative to where the :command:`snapcraft` command is executed. This will typically mean creating a *hooks* directory in the same directory that contains the :file:`snapcraft.yaml` file for your project. The following, for example, shows the location of a configure hook executable:

::

   .
   └── snap
       ├── hooks
       │   └── configure
       └── snapcraft.yaml

Hook executables in ``snap/hooks`` are automatically copied into the snap during Snapcraft’s ``prime`` step (see :ref:`Parts lifecycle <parts-lifecycle>` for details).

2. From within snapcraft.yaml
-----------------------------

A part within :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` can also generate and install hook executables into ``$SNAPCRAFT_PART_INSTALL/snap/hooks/``. These are automatically copied into the snap during the ``prime`` step.

.. code:: yaml

   configure-hook:
       plugin: dump
       source: configure-again/
       organize:
           configure: snap/hooks/configure

Plugs
-----

By default, hooks run with no plugs. If a hook needs more privileges, it can use the top-level ``hooks`` attribute in ``snapcraft.yaml`` to request plugs:

.. code:: yaml

   hooks: # Top-level YAML attribute, parallel to `apps`
     configure: # Hook name, corresponds to executable name
       plugs: [network] # Or any other plugs required by this hook

Hooks are called with no parameters. See `Using the snapctl tool <https://snapcraft.io/docs/using-the-snapctl-tool>`__ for details on the internal command they can use to provide and retrieve data to and from snapd.
