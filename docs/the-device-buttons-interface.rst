.. 8598.md

.. _the-device-buttons-interface:

The device-buttons interface
============================

``device-buttons`` allows read and write access to device buttons exposed as input events. Such buttons can be defined using ``gpio-keys`` inside the device tree bindings. Consult kernel documentation on `gpio-keys <https://www.kernel.org/doc/Documentation/devicetree/bindings/input/gpio-keys.txt>`__ for more details.

The interface can access ``/dev/input/event*`` devices that are udev marked with ``ID_INPUT_KEY=1`` but are not keyboards (``ID_INPUT_KEYBOARD!=1``).

**Auto-connect**: no

Requires snapd version *2.37+*.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
