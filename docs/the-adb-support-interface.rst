.. 9720.md

.. _the-adb-support-interface:

The adb-support interface
=========================

``adb-support`` allows a snap to operating the Android Debug Bridge service, providing privileged access to an Android device.


.. _the-adb-support-interface-example:

Example
-------

`guiscrcpy <https://snapcraft.io/guiscrcpy>`__ uses *adb-support* to help share an Android screen on a Linux desktop.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-adb-support-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no

Requires snapd version *2.36+*.


.. _the-adb-support-interface-heading-code:

Code examples
~~~~~~~~~~~~~

The adb-support interface is used in the *guiscrcpy* snap: https://github.com/srevinsaju/guiscrcpy/blob/master/snap/snapcraft.yaml

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/adb_support.go
