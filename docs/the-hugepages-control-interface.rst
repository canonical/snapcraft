.. 26501.md

.. _the-hugepages-control-interface:

The hugepages-control interface
===============================

The ``hugepages-control`` interface allows controlling `HugePages <https://www.kernel.org/doc/Documentation/vm/hugetlbpage.txt>`__, a Linux kernel feature that enables memory blocks to be managed in larger page sizes.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

Developer details
-----------------

**Auto-connect**: no

This interface assumes that *huge pages* are mounted at either:

- ``/dev/hugepages`` (Debian, Ubuntu)
- ``/run/hugepages`` (various other distributions)

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/hugepages_control_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/hugepages_control.go
