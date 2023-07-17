.. 7777.md

.. _the-classic-support-interface:

The classic-support interface
=============================

The ``classic-support`` interface sets special permissions for the `classic snap <https://snapcraft.io/classic>`__, effectively giving device ownership to its connected snaps.

This interface is intended to be used only with :term:`Ubuntu Core`.

Requires snapd version *2.23+*.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-classic-support-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/classic_support_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/classic_support.go
