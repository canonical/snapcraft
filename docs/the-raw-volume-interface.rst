.. 14578.md

.. _the-raw-volume-interface:

The raw-volume interface
========================

``raw-volume`` provides access to a specific disk partition. This interface is restricted because it provides privileged access to device partitions.

The slot is intended to be implemented by a gadget snap and is not provided by the core system snap.

**Auto-Connect**: no

**Attributes**:

* ``path`` (slot): path to device partition e.g. ``/dev/mmcblk0p1``

Requires snapd version *2.43+*.

To use a disk partition, the snap developer must add ``plugs: [ raw-volume ]`` to a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`. The snap user can then access a specific disk partition with an :ref:`interface connection <interface-management-manual-connections>`.

Use ``snap interface raw-volume`` to see which disk partitions are available on the system for snaps to use:

.. code:: bash

   $ snap interface raw-volume
   name:    raw-volume
   summary: allows access to specific disk partition
   slots:
     - foo:mmcblk0p1
     - foo:mmcblk0p2

Once connected, the consuming snap can use the device via the path specified by the connected slot.

   ⓘ This is a snap interface. See :ref:`interface-management` and :ref:`supported-interfaces` for further details on how interfaces are used.
