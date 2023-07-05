.. 26569.md

.. _the-snap-refresh-control-interface:

The snap-refresh-control interface
==================================

The ``snap-refresh-control`` interface allows extended control, via :ref:`snapctl <using-the-snapctl-tool>`, of refreshes targeting the snap.

**This interface and the full set of features it requires to function are currently under development.**

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-snap-refresh-control-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

``snap-refresh-control`` is a marker interface (with no associated AppArmor or Seccomp rules).

Currently it allows connected snaps to execute ``snapctl refresh --proceed`` to unblock pending refreshes outside of the context of the ``gate-auto-refresh`` hook. This interface should be used with caution.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/snap_refresh_control_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/snap_refresh_control.go
