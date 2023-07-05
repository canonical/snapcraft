.. 3795.md

.. _supported-snap-hooks:

Supported snap hooks
====================

A hook is an executable file that runs within a snap’s confined environment when a certain action occurs.

Common examples of actions requiring hooks include:

-  **notifying a snap that something has happened** Example: If a snap has been upgraded, the snap may need to trigger a scripted migration process to port an old data format to the new one.

-  **notifying a snap that a specific operation is in progress** Example: A snap may need to know when a specific interface connects or disconnects.

A hook is defined as an executable within a snap’s ``meta/hooks/`` directory, and consequently, also within ``snap/hooks/`` when building with *snapcraft*. See :ref:`Snapcraft hook support <snapcraft-hook-support>` for more information.

The filename of the executable is based on the name of the hook. If this file exists, *snapd* will execute the file when required by that hook’s action.

The following hooks are currently implemented:

- `configure hook <supported-snap-hooks-the-configure-hook_>`__
- `default-configure hook <supported-snap-hooks-default-configure_>`__
- `full-disk-encryption hook <supported-snap-hooks-fde_>`__
- `gate-auto-refresh <supported-snap-hooks-gate-auto-refresh_>`__
- `install hook <supported-snap-hooks-install_>`__
- `install-device hook <supported-snap-hooks-install-device_>`__
- `interface hooks <supported-snap-hooks-interface_>`__
- `prepare-device hook <supported-snap-hooks-prepare-device_>`__
- `pre-refresh hook <supported-snap-hooks-pre-refresh_>`__
- `post-refresh hook <supported-snap-hooks-post-refresh_>`__
- `remove hook <supported-snap-hooks-remove_>`__

   ⓘ **Default shell environment**: A hook script can only assume a POSIX-compliant shell environment for its execution. If your script needs a specific shell, such as *Bash* or *Zsh*, it needs to be explicitly declared within the script’s *hashbang* header (``#!/bin/bash``, for example). Your snap also needs to ensure your chosen shell is available.

Accessing resources
-------------------

If a hook requires access to system resources outside of a snap’s confined environment, it will need to use :ref:`slots and plugs <interface-management-slots-plugs>` via the :ref:`interface <interface-management>` mechanism to access those resources.

When using *Snapcraft* to build the snap, the interface definition will go inside :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`, and the *snapcraft* command create a :ref:`snap.yaml <the-snap-format>` within the snap to hold the required metadata.

For example, the following excerpt registers an *install* hook making use of a :ref:`network <the-network-interface>` plug:

.. code:: yaml

   apps:
       ...

   hooks:
       install:
           plugs: [network]

Hooks are called with no parameters. When a hook needs to request or modify information within *snapd*, they can do so via the *snapctl* tool, which is always available within a snap’s environment. See `Using the snapctl tool <https://snapcraft.io/docs/using-the-snapctl-tool>`__ for further details.

A hook is executed as a single transaction, where a transaction object holds all the configuration changes for that hook. These changes are invisible to the running system until the hook completely finishes. This allows for changes to be rolled back or *unset* if errors occur during the execution of a hook.

--------------


.. _supported-snap-hooks-the-configure-hook:

The configure hook
------------------

The ``configure`` hook is called every time one the following actions happen:

-  initial snap installation
-  snap refresh
-  whenever the user runs ``snap set|unset`` to change a configuration option

..

   ⓘ Note that this hook will *not* get called when the snap *itself* changes configuration options using ``snapctl get|set|unset``.

The hook should use ``snapctl get`` to retrieve configuration values from snapd. If the hook exits with a non-zero status code, the configuration will not be applied.

For example, given the following command:

.. code:: bash

   $ snap set mysnap username=foo password=bar

The ``configure`` hook located within the *mysnap* snap at ``meta/hooks/configure`` would be called to apply the configuration changes, if necessary.

The hook might look similar to:

.. code:: sh

   #!/bin/sh -e

   username="$(snapctl get username)"
   password="$(snapctl get password)"

   if [ -z "$username" -o -z "$password" ]; then
       echo "Username and password are required."
       exit 1
   fi

   mkdir -m 0600 $SNAP_DATA/options
   echo "username: $username" > $SNAP_DATA/options/credentials
   echo "password: $password" >> $SNAP_DATA/options/credentials

The same hook can also modify the configuration of a snap within the context of the current transaction. This is accomplished using ``snapctl set`` and ``snapctl unset``. For more information see `Adding Snap configuration <https://snapcraft.io/docs/adding-snap-configuration>`__ and `Using the snapctl tool <https://snapcraft.io/docs/using-the-snapctl-tool>`__.

   ⓘ Note that configuration options do not need to be defined anywhere. ``snapctl set`` and ``snap set`` will accept any (valid) option name.


.. _supported-snap-hooks-default-configure:

The default-configure hook
--------------------------

The default-configure-hook is an optional extension to the :ref:`configure hook <supported-snap-hooks-the-configure-hook>` that executes only on snap installation and *before* services are started to provide access to the default configuration values stored in a device’s :ref:`gadget snap <gadget-snaps>`.

The default-configure hook should be located within ‘meta/hooks’ and requires a configure hook to be present. A missing configure hook will result in an error.

The hook should use ``snapctl get`` to retrieve default configuration options and ``snapctl set`` and ``snapctl unset`` to create/modify options. If the hook exits with a non-zero status code, the default configuration provided by the gadget snap as well as modifications by the hook will not be applied.

Example default-configure hook:

.. code:: sh

   #!/bin/sh -e

   DEFAULT_GADGET_OPTION=”123”

   gadget_option="$(snapctl get gadget_option)"
   if [ -z "$gadget_option" ]; then
   gadget_option="$DEFAULT_GADGET_OPTION"
   fi

   mkdir -m 0600 $SNAP_DATA/options
   echo "option: $gadget_option" > $SNAP_DATA/options/gadget

For more information see `Adding Snap configuration <https://snapcraft.io/docs/adding-snap-configuration>`__ and `Using the snapctl tool <https://snapcraft.io/docs/using-the-snapctl-tool>`__.


.. _supported-snap-hooks-fde:

The full-disk-encryption hook
-----------------------------

`Ubuntu Core 20 <https://ubuntu.com/core/docs/uc20/>`__ (UC20) uses `full disk encryption <https://ubuntu.com/core/docs/uc20/full-disk-encryption>`__ (FDE) whenever the hardware allows, protecting both the confidentiality and integrity of a device’s data when there’s physical access to a device, or after a device has been lost or stolen.

Creating a verifiable boot process on a non-standard (non-UEFI+TPM platform) FDE platform, such as a Raspberry Pi or other ARM devices, is board-specific and will typically involve creating custom gadget and kernel snaps. UC20, however, does provide a helper mechanism, via a hook interface, to ensure the integrity of any subsequently executed or accessed data.

See `UC20 full-disk-encryption hook interface <https://snapcraft.io/docs/uc20-uc22-full-disk-encryption-hook-interface>`__ for details on how this hook is implemented.


.. _supported-snap-hooks-gate-auto-refresh:

The gate-auto-refresh hook
--------------------------

The gate-auto-refresh hook is executed by snapd for every snap that will be updated with the next automatic refresh. It’s also executed for every snap that is dependent on a snap that will be updated.

This hook is capable of executing the snapctl refresh command with 3 specific arguments, hold, proceed and pending.

This feature is currently considered experimental. See `Refresh control <https://snapcraft.io/docs/refresh-control>`__ for more details.


.. _supported-snap-hooks-install:

The install hook
----------------

The ``install`` hook is called upon initial install only, i.e. it’s not called on subsequent refreshes.

The hook is executed before starting snap services (if it has any) and before the ``configure`` hook. The install hook is the place for one-time actions, such as an early initialisation of a resource when installed for the first time.


.. _supported-snap-hooks-install-device:

The install-device hook
-----------------------

This hook is supported in Ubuntu Core 20 and subsequent releases.

See `Installation process <https://ubuntu.com/core/docs/uc20/installation-process#supported-snap-hooks-heading--install-device>`__ in the Ubuntu Core documentation for more details.


.. _supported-snap-hooks-interface:

The interface hooks
-------------------

Interface hooks are executed when an interface is either connected or disconnected via the interface’s plugs and slots mechanism.

They can be used to read or write attributes from a connection and, for example, acquire new resources, update internal options or update databases.

For further details, see :ref:`Interface hooks <interface-hooks>`.


.. _supported-snap-hooks-prepare-device:

The prepare-device hook
-----------------------

This hook is only supported in gadget snaps.

See :ref:`The gadget snap <gadget-snaps>` documentation for more details.


.. _supported-snap-hooks-pre-refresh:

The pre-refresh hook
--------------------

The ``pre-refresh`` hook is called whenever the snap gets refreshed.

This hook is executed for the already installed revision of the snap with its services still running (if the snap has any services) and before switching to the newly installed revision.

This hook is a good place for any maintenance or cleanup actions that prepare the snap for switching to the new revision. It’s also a good place to test whether a refresh will succeed, because if the test causes the hook to fail, the refresh will not proceed.


.. _supported-snap-hooks-post-refresh:

The post-refresh hook
---------------------

The ``post-refresh`` hook is similar to ``pre-refresh`` (above) in that it is called whenever the snap gets refreshed.

This hook is executed for the newly installed snap, before starting new services (if applicable). This hook is a good place for any extra actions that need to be performed for the new revision of the snap. It’s also a good place to test whether a refresh has succeeded, because if the test causes the hook to fail, the refresh will be rolled-back and the original state of the snap restored.


.. _supported-snap-hooks-remove:

The remove hook
---------------

The ``remove`` hook is called when the last revision of the snap gets removed from the system.

This hook is executed after stopping the services of the snap (if the snap has any services), therefore it’s useful for any custom cleanup logic.
