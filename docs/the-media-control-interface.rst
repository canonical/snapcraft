.. 26504.md

.. _the-media-control-interface:

The media-control interface
===========================

The ``media-control`` interface permits access to media control devices and Video4Linux (V4L) devices via the Linux kernel’s `Media Controller API <https://www.kernel.org/doc/html/latest/userspace-api/media/mediactl/media-controller.html>`__.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

The kernel media controller API allows connecting and configuring media hardware subsystems. These operations should be considered privileged since the driver assumes trusted input, and therefore this interface requires manual connection.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/media_control_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/media_control.go
