.. 35421.md

.. _hardware-io-interfaces:

Hardware IO interfaces
======================

Hardware IO (input/output) interfaces, including the :ref:`serial-port <the-serial-port-interface>`, :ref:`gpio <the-gpio-interface>` and :ref:`i2c <the-i2c-interface>` interfaces, are designed to be used on devices running :term:`Ubuntu Core`. These interfaces are driven from a device’s :ref:`gadget snap <gadget-snaps>` which is used to define and configure a device’s system properties.

This approach is more robust because it allows the gadget snap providing the slot to centralise and arbitrate the connection conditions. These conditions include which other snaps, identified by their snap ID, can connect to the slots the gadget offers and, consequently, gain privileged access to the hardware. For the application snap, usually no change is required other than to declare and use an appropriately-configured plug.


.. _hardware-io-interfaces-considerations:

Interface considerations
------------------------

The extent of access an interface has is granted through both *connection permissions* and the specifics of the *interface connections* being requested.

1. **Connection permissions**: :ref:`auto-connect <the-interface-auto-connection-mechanism>` \| :ref:`privileged <interface-management>` \| :ref:`super-privileged <super-privileged-interfaces>` Connection requirements are dependent on which store a developer is using.

   -  Global :term:`Snap Store`: privileged and super-privileged interfaces require store approval because of the level of trust and permissiveness these interfaces have, which is also why certain interfaces need certain oversight. See :ref:`Permission requests <permission-requests>` for further details.
   -  :term:`Dedicated Snap Store`: trust and permissiveness are now the responsibility of the store owner, and many privileged interface connections can be self-served and defined within the dedicated snap store and the device context.

2. **Interface connections**: hardware IO interfaces \| app-provided interfaces \| other interfaces

   -  **Hardware IO interfaces**: These require either a :ref:`slot <interface-management-slots-plugs>` to be defined by a device’s *gadget snap* or an interface with `Hotplug support <https://snapcraft.io/docs/hotplug-support>`__, in which case the slot appears from the system snap.

      -  An unconstrained :ref:`auto-connection <the-interface-auto-connection-mechanism-autoconnect>` cannot be used because there may be *many slots of a given interface*, resulting in ambiguity that requires an extensive set of store rules to manage and maintain.
      -  Each plug should therefore be connected to a slot, for example:

         -  green led plug on app => green led slot on gadget
         -  red led plug on app => red led slot on gadget

      -  This kind of 1-to-1 connections can usually be established via :ref:`slot rules in the snap-declaration <the-interface-auto-connection-mechanism>` for the gadget.

   -  **App-provided interfaces**: slots are defined by apps, or occasionally from the gadget snap,

      -  May require access, such as from the :ref:`content <the-content-interface>` or :ref:`shared-memory <the-shared-memory-interface>` interfaces.
      -  A slot might may be provided by the system snap to cover the case of an equivalent system service, such as :ref:`audio-playback <the-audio-playback-interface>`
      -  the slot might be :ref:`super-privileged <super-privileged-interfaces>`

   -  **Other interfaces**: For more system level access, slots are provided by the system snap.


.. _hardware-io-interfaces-code-examples:

Code examples
~~~~~~~~~~~~~

The `gadget snap <https://github.com/snapcore/pi-gadget/tree/20-arm64>`__ definition for the reference `Raspberry Pi Ubuntu Core <https://ubuntu.com/core/docs/install-raspberry-pi>`__ image contains interface definitions for various hardware IO interfaces on the system, including slots for each specific GPIO pin, i2c connections, the Bluetooth serial port, and the generic serial ports:

.. code:: yaml

   slots:
     bcm-gpio-0:
       interface: gpio
       number: 0
     bcm-gpio-1:
       interface: gpio
       number: 1
     bcm-gpio-2:
       interface: gpio
       number: 2
   [...]
     i2c-0:
       interface: i2c
       path: /dev/i2c-0
   [...]
     bt-serial:
       interface: serial-port
       path: /dev/ttyAMA0
   [...]
     serial0:
       interface: serial-port
       path: /dev/ttyS0
     serial1:
       interface: serial-port
       path: /dev/ttyS1

On a Raspberry Pi, the above hardware IO interfaces are accessible to apps from the system snap without requiring any further configuration.
