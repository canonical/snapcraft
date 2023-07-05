.. 20179.md

.. _the-interface-auto-connection-mechanism:

The interface auto-connection mechanism
=======================================

Interfaces allow (or deny) access to a resource outside of a snap’s confinement, but a they’re also designed to be convenient for users.

Part of this convenience is whether an interface is permitted to be automatically connected when a snap installed, or whether a snap can request to include certain super-privileged interfaces to begin with.


.. _the-interface-auto-connection-mechanism-autoconnect:

Auto-connections
----------------

The snapd :ref:`interface auto-connection <interface-management-auto-connections>` mechanism has been designed to remove the need for a manual connection when:

-  An interface is commonly required and has no or low security implications
-  A snap provides, or accesses, libraries or other content vital to the operation of a snap
-  There is a need to connect snaps that are designed to work together
-  Store-set policy applies to let a given snap use an interface that would not be generally auto-connected.


.. _the-interface-auto-connection-mechanism-process:

Auto-connection process
~~~~~~~~~~~~~~~~~~~~~~~

Whenever a snap is installed or refreshed, snapd will review its unconnected plugs and their candidate slots for auto-connection potential. It does the same for its slots and candidate plugs.

If there is exactly one candidate slot for a plug then an interface connection is automatically performed. The requirement for exactly one candidate can be changed.

For an interface, the candidate plug and slot pairs are determined using constraints-based rules. The rule language can either express allowing or denying an auto-connection, with the latter taking precedence.

Rules are either built-in to snapd or conveyed via assertions (signed documents) that correspond to a given snap (snap-declarations). The rules are scoped and attached to the slot or plug side of an interface at either one of those two levels:

-  the built-in level
-  the snap level

To simplify reasoning about their effect, the rules from the different levels/scopes are not merged but rather prioritised according to level:

-  snap plug (from the snap-declaration of the snap with the plug)
-  snap slot (from the snap-declaration of the snap sporting the slot)
-  built-in plug level
-  built-in slot level

From snap plug to the built-in slot level, it’s the level that first provides any rules that determines the rules used and considered. This ordering reflects a scale from the most specific usage perspective, snap plug side consumption, to the most general, builtin slot rules.

Built-in rules embody the general policy of whether and when an interface should be auto-connected, and as such, these are documented with the interface. Consequently, snap-declaration rules express store-set policy.

A user can also issue “snap connect” and “snap disconnect” commands. In particular, a manual “snap disconnect” of an auto-connection will inhibit a subsequent refresh from re-establishing the auto-connection.


.. _the-interface-auto-connection-mechanism-constraints:

Auto-connection constraints
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following is an example built-in auto-connection rule (attached to the slot-side) for the content interface:

.. code:: yaml

   slots:
     content:
       allow-auto-connection:
         plug-publisher-id:
           - $SLOT_PUBLISHER_ID
         plug-attributes:
           content: $SLOT(content)

The above example permits an auto-connection if:

-  the publisher of both the slot and plug side snaps is the same
-  the “content” attribute on the plug and slot match

The rule language allows us to deny or allow auto-connections with the following constraints:

-  attributes values on the plug and slot (based on regular expressions or special expressions like $SLOT(content) in the example)
-  snap type or the snap-id of the plug or slot snap
-  publisher of the slot or plug snap
-  actual names of the plug or slot
-  whether the device is a classic system or not
-  the actual store in use or device model (for IoT use-cases)

Further lists of constraints or values can be used in the rule language to express alternation (logical OR).

   ℹ The same language rules can be used to define rules other than interface auto-connections, including default policy for ordinary connections and installations, and to override policy for the more sensitive interfaces.


.. _the-interface-auto-connection-mechanism-default-provider:

Auto-connection with a default-provider
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A content interface plug can specify a default-provider. This is the name of a snap that can be installed to fulfil the plug’s need. If there isn’t a slot with the content label of the plug on the system the specified snap will be installed before the snap with the plug. However, if there is a slot on the system that already has the content label, any default-provider specified by a content interface plug is ignored. If the specified snap is already installed but there is no slot with the content label, the specified snap will be upgraded instead under the assumption that a newer version may start providing the label.

The default-provider mechanism triggers the installation or upgrade of the provider snap as described but it does not include an implied connection. The auto-connection will only happen by means of and if it is permitted by the general auto-connection mechanism.
