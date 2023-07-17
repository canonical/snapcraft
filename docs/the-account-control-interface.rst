.. 7746.md

.. _the-account-control-interface:

The account-control interface
=============================

``account-control`` allows managing non-system user accounts on `Ubuntu Core <https://ubuntu.com/core/docs>`__ systems.

This interface enables the management of the ``extrausers`` table in the Name Service Switch (NSS) databases on Ubuntu Core to manage both non-system unprivileged and privileged users and groups.

The interface **does not** allow the management of users and groups for the system NSS databases in */etc*.

Due to the privileged nature of access enabled by this interface, its use is reserved exclusively for “management snaps” from :term:`brand stores`.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-account-control-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no


.. _the-account-control-interface-code:

Code examples
~~~~~~~~~~~~~

The account-control interface is used in the *usbtop* snap to help monitor USB traffic: https://github.com/ogra1/usbtop/blob/master/snap/snapcraft.yaml

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/account_control.go
