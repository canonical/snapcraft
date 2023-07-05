.. 7908.md

.. _the-raw-usb-interface:

The raw-usb interface
=====================

``raw-usb`` allows access to all connected USB devices via a raw interface.

**Auto-connect**: no

Requires snapd version *2.18+*.

**NOTE:** The use of this interface only extends the snapd’s own sandbox confinement, it will NOT supersede the classic Unix file permission model so the user still needs to have sufficient ``r`` or ``w`` permission to the device node by either run the snap command as root or have a designated udev rule that grants the permission.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
