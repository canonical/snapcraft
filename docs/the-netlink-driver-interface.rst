.. 25485.md

.. _the-netlink-driver-interface:

The netlink-driver interface
============================

The ``netlink-driver`` interface allows a kernel module to expose itself to user-space via the Netlink protocol, typically to transfer information between the kernel and user-space processes.

See also :ref:`netlink-audit <the-netlink-audit-interface>` and :ref:`netlink-connector <the-netlink-connector-interface>`.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

Further confinement for particular families/protocols is implemented via Seccomp filtering network Netlink.

Requires snapd version *2.51.1+*.


Code examples
~~~~~~~~~~~~~

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/netlink_driver.go
