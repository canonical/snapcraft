.. 30982.md

.. _the-acrn-interface:

The acrn interface
==================

The ``acrn`` interface allows access to, and control of, user virtual machines using the `ACRN hypervisor <https://projectacrn.org/>`__.

**This interface is primarily intended to be used with**\ :term:`Ubuntu Core`\ **devices.**

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-acrn-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: no

Code examples
-------------

The following (third-party) repository contains recipes to create snap packages for ACRN: https://github.com/gvancuts/acrn-snap

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/acrn_support_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/acrn_support.go
