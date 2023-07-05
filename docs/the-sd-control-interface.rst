.. 25489.md

.. _the-sd-control-interface:

The sd-control interface
========================

The ``sd-control`` interface allows for the management and control of SD cards on certain devices using the DualSD driver.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-sd-control-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

The main DualSD device node (``/dev/DualSD``) is used to control certain aspects of SD cards on the system.

Requires snapd version *2.51.3+*.


.. _the-sd-control-interface-heading-code:

Code examples
~~~~~~~~~~~~~

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/sd_control.go
