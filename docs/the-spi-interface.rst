.. 7916.md

.. _the-spi-interface:

The spi interface
=================

``spi`` allows access to a specific Serial Peripheral Interface (SPI) controller. This snap interface is restricted because it provides privileged access to SPI hardware.

**Auto-Connect**: no

**Attributes**:

* ``path`` (slot): path to the specific SPI device node e.g. ``/dev/spidev0.0``

Snaps that want to consume an SPI device simply use ``plugs: [ spi ]``. The SPI device to connect to is specified during the interface connection.

To see which SPI slots are available on your system, run ``snap connections --all`` to show all the possible slots on your system:

::

   $ snap connections --all | grep spi
   spi         -        pi:spidev0              -
   spi         -        pi:spidev1              -

Once connected, the consuming snap can use the device via the path specified by the connected slot.

Requires snapd version *2.28+*.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
