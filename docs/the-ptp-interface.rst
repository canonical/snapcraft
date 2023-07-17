.. 26565.md

.. _the-ptp-interface:

The ptp interface
=================

The ``ptp`` interface allows access to the Precision Time Protocol (PTP) `Hardware Clock subsystem <https://www.kernel.org/doc/Documentation/ptp/ptp.txt>`__ in the Linux kernel, enabling the clock to be synced to sub-100 nanoseconds.

This interface is primarily intended to be used with :term:`Ubuntu Core`.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-ptp-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no

See `sysfs-ptp <https://github.com/torvalds/linux/blob/master/Documentation/ABI/testing/sysfs-ptp>`__ for device *sysfs* location and configuration details.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/ptp_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/ptp.go
