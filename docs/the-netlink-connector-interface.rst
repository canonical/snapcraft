.. 7879.md

.. _the-netlink-connector-interface:

The netlink-connector interface
===============================

The ``netlink-connector`` interface allows communication through the kernel Netlink connector.

See also :ref:`netlink-driver <the-netlink-driver-interface>` and :ref:`netlink-audit <the-netlink-audit-interface>`.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

As NETLINK_CONNECTOR is not finely mediated and app-specific, use of this interface allows communications via all Netlink connectors. See `Kernel connector`_ (on kernel.org) for further details.

Requires snapd version *2.26+*.


Code examples
~~~~~~~~~~~~~

The snap of the ``usbtop`` kernel module, used to monitor the bandwidth of USB buses and devices, uses the *netlink-audit* interface: `https://github.com/ogra1/usbtop/blob/master/snap/snapcraft.yaml <https://github.com/ogra1/usbtop/blob/3743b5a55e6df70e6dd95292121279f1013ba570/snap/snapcraft.yaml#L50>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/netlink_connector.go
