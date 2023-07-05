.. 7846.md

.. _the-iio-interface:

The iio interface
=================

``iio`` enables access to a specific IIO (Industrial I/O) device. This interface is restricted because it provides privileged access to IIO hardware.

**Auto-Connect**: no

**Attributes:**

* ``path`` (slot): path to IIO device node e.g. ``/dev/iio:device0``

Available IIO devices for the system can be seen with ``snap connections``. Once connected, the consuming snap can use the device via the path specified by the connected slot.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
