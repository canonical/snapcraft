.. 7842.md

.. _the-i2c-interface:

The i2c interface
=================

``i2c`` provides access to a specific I2C controller. This interface is restricted because it provides privileged access to hardware devices.

**Auto-Connect**: no

**Attributes**:

* ``path`` (slot): path to i2c device node e.g. ``/dev/i2c-0``

:ref:`Hardware IO interfaces <hardware-io-interfaces>` covers some general considerations common to these kinds of devices.

To use a i2c device, the snap developer must add ``plugs: [ i2c ]`` to a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`. The snap user can then access a specific i2c device with an :ref:`interface connection <interface-management-manual-connections>`.

Use ``snap interface i2c`` to see which i2c devices are available on the system:

.. code:: bash

   $ snap interface i2c
   name:    i2c
   summary: allows access to specific I2C controller
   slots:
     - pi:i2c-0
     - pi:i2c-1
     - pi:i2c-2

Once connected, the consuming snap can use the device via the path specified by the connected slot.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
