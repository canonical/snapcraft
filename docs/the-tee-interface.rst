.. 26573.md

.. _the-tee-interface:

The tee interface
=================

The ``tee`` interface permits access to Trusted Execution Environment (TEE) devices via the `TEE subsystem <https://www.kernel.org/doc/html/latest/staging/tee.html>`__ in the Linux kernel.

This interface is primarily intended to be used with :term:`Ubuntu Core`.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

Intended for snaps needing to access the the TEE subsystem over ``/dev/tee[0-9]*``, ``/dev/teepriv[0-0]*`` or the Qualcomm equivalent *qseecom* (Qualcomm Secure Execution Environment Communicator) at ``/dev/qseecom``.


Code examples
-------------

The official `Ubuntu Core gadget snap <https://github.com/kubiko/imx8m-gadget>`__ for the `i.MX8M Mini Evaluation Kit <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/evaluation-kit-for-the-i-mx-8m-mini-applications-processor:8MMINILPD4-EVK>`__ uses this interface: https://github.com/kubiko/imx8m-gadget/blob/imx8mm-evk/snap/snapcraft.yaml

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/tee_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/tee.go
