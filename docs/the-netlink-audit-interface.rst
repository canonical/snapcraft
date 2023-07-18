.. 7878.md

.. _the-netlink-audit-interface:

The netlink-audit interface
===========================

The ``netlink-audit`` interface allows access to the kernel part of the Linux Audit Subsystem through Netlink.

See also :ref:`netlink-driver <the-netlink-driver-interface>` and :ref:`netlink-connector <the-netlink-connector-interface>`.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


.. _the-netlink-audit-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no

Requires snapd version *2.26+*.


.. _the-netlink-audit-interface-heading-code:

Code examples
~~~~~~~~~~~~~

The snap of the ``usbtop`` kernel module, used to monitor the bandwidth of USB buses and devices, uses the *netlink-audit* interface: `https://github.com/ogra1/usbtop/blob/master/snap/snapcraft.yaml <https://github.com/ogra1/usbtop/blob/3743b5a55e6df70e6dd95292121279f1013ba570/snap/snapcraft.yaml#L50>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/netlink_audit.go
