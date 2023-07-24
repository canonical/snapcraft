.. 8214.md

.. _interface-hooks:

Interface hooks
===============

Interface hooks are :ref:`hooks <supported-snap-hooks>` that can be executed when an :ref:`interface <interfaces>` is either connected or disconnected via
the interface's plugs and slots mechanism. They can be used to read or write attributes from a connection and, for example, acquire new resources, update internal options or update databases.

.. note::

   An interface consists of a connection between a plug and a slot. The slot is the provider of the interface while the plug is the consumer, and a slot can support multiple plug connections. For more details, see :ref:`Interface management <interface-management>`.

There are two types of hook: connect hooks are run when a connection is made, disconnect hooks are run when a disconnection occurs.

Connect hooks
-------------

When a plug connects to a slot, the following hooks are executed sequentially *before* the interface connection occurs:

1. ``prepare-plug-<plugname>`` for the snap on the plug-side
2. ``prepare-slot-<slotname>`` for the snap on the slot-side

After the interface connection has been created and the security permissions have been updated, the following two hooks are then executed:

3. ``connect-slot-<slotname>`` for the snap on the slot-side
4. ``connect-plug-<plugname>`` for the snap on the plug-side

If an interface connection fails, all changes are undone and an *undo* hook is run for any interface hooks executed prior to the failure. These undo hooks are executed in reverse order:

#. ``disconnect-plug-<plugname>``
#. ``disconnect-slot-<slotname>``
#. ``unprepare-slot-<slotname>``
#. ``unprepare-plug-<plugname>``

Interface hooks can read the attributes of the affected plug and slot by using the *snapctl* command, as defined by the snap's :ref:`snap.yaml <the-snap-format>` file.

When executed by slot hooks:

- ``snapctl get :<slotname> <attribute name>`` to read attributes of the local slot
- ``snapctl get --plug :<slotname> <attribute name>`` to read attributes of the plug

When executed by plug hooks:

- ``snapctl get :<plugname> <attribute name>`` to read attributes of local plug
- ``snapctl get --slot :<plugname> <attribute name>`` to read attributes of the slot

The plug and slot hooks do not know the corresponding slot and plug names on the other end of the connection. For this reason, the above syntax uses ``:<slotname>`` in the slot hooks and ``:<plugname>`` in the plug hooks. It’s the ``--plug`` or ``--slot`` flag that tells *snapctl* which end of the connection to read attributes from.

In addition to the plug and slot attributes defined in *snap.yaml*, ``prepare-plug-`` and ``prepare-slot-`` hooks can create attributes dynamically with the *snapctl* command:

In the *prepare-slot-<slotname>* hook:

- ``snapctl set :<slotname> slotattribute=value`` to create an attribute for the slot side

In the *prepare-plug-<plugname>* hook:

- ``snapctl set :<plugname> plugattribute=value`` to create an attribute for the plug side

Attributes can only be created in *prepare-* hooks, and only as long as their names do not clash with attributes already defined statically in snap.yaml for the given plug/slot. This means overwriting or overloading statically defined attributes is not allowed.

Disconnect hooks
----------------

Disconnect hooks are executed upon manual disconnection or automatically when a snap on either end of a connection is completely removed from the system.

Disconnect hooks are executed in the following order:

1. ``disconnect-slot-<slotname>`` for the snap on the slot-side
2. ``disconnect-plug-<plugname>`` for the snap on the plug-side

Disconnect hooks can read the attributes of both the plug and slot side of the connection via the *snapctl* command, as described in the `Connect hooks`_ section above.

If an interface disconnection fails, an *undo* hook is run for any interface hooks executed prior to the failure and the connection is restored. These undo hooks are executed in reverse order:

-  ``connect-plug-<plugname>``
-  ``connect-slot-<slotname>``
