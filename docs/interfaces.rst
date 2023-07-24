.. 35928.md

.. _interfaces:

Interfaces
==========

By default, snaps with strict confinement are only able to access a limited set of resources outside the environment they run in. Snaps can only access resources from the system and other snaps via interfaces that describe the resources they provide.

The creator of a snap selects the interfaces the snap requires to function
correctly. Common interfaces include those that provide access to the
:ref:`network <the-network-interface>`, :ref:`desktop features
<the-desktop-interfaces>` and the :ref:`sound system
<the-pulseaudio-interface>`.

An interface must be active to access resources. Inactive interfaces cannot access the resources that the interface provides. An interface becomes active when it is connected.

Connecting interfaces
---------------------

A connection between snaps is made when a plug from a snap needing a resource
is connected to a slot in another snap providing that resource.
A analogy to this is plugging an appliance into an electrical outlet that
provides the power it needs.

.. figure:: https://assets.ubuntu.com/v1/59c290a8-snapd-interfaces.png
   :alt: How an interfaces uses a plug and a slot

Connections are made either automatically at install time or manually, depending on their function. The desktop interface is connected automatically, for instance, whereas the camera interface is not. See the *Auto-connect* column in the table of :ref:`supported-interfaces` for details on whether an interface automatically connects or not.

As with `classic confinement </t/33649>`__, a snap’s publisher can request an *assertion* to automatically connect an otherwise non-auto-connecting interface. For example, the *guvcview* snap `requested <https://forum.snapcraft.io/t/auto-connect-request-for-the-guvcview-brlin-snap/6042>`__ the camera interface be automatically-connected when the snap is installed.

If a snap is upgraded and includes a new assertion, the user will still need to connect the interface manually. Similarly, if an installed classic snap is upgraded to use strict confinement, its interfaces won’t be automatically configured.

Getting the interfaces for a snap
---------------------------------

Use the ``snap connections`` command to see which interfaces a snap needs, and which are currently connected:

.. code:: bash

   $ snap connections vlc
   Interface         Plug                  Slot               Notes
   camera            vlc:camera            -                  -
   desktop           vlc:desktop           :desktop           -
   desktop-legacy    vlc:desktop-legacy    :desktop-legacy    -
   home              vlc:home              :home              -
   mount-observe     vlc:mount-observe     -                  -
   [...]

In the above example, we can see that the ``vlc:camera`` interface is disconnected because it has an empty *Slot* entry.

See :ref:`interface-management` for further interface details, including how to disconnect interfaces and make manual connections, and `Security policy and sandboxing <https://forum.snapcraft.io/t/security-policy-and-sandboxing/554>`__ for more information on how confinement is implemented.
