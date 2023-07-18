.. 26498.md

.. _the-fpga-interface:

The fpga interface
==================

The ``fpga`` interface allows access to the `FPGA subsystem <https://www.kernel.org/doc/html/latest/driver-api/fpga/index.html>`__.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

**Allow-installation**: yes

Devices: ``/dev/fpga[0-9]* rw,``

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/fpga_test.go

The source code for the interface is in the snapd repository:https://github.com/snapcore/snapd/blob/master/interfaces/builtin/fpga.go
