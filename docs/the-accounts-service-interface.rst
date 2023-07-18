.. 7802.md

.. _the-accounts-service-interface:

The accounts-service interface
==============================

``accounts-service`` allows communication with the *accounts* service, such as `GNOME Online Accounts`_.


Example
-------

This interface automatically connected by the `ONLY OFFICE Document Server`_ snap to provide better integration with pre-configured accounts.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no


Code examples
~~~~~~~~~~~~~

The snapcraft.yaml for ONLY OFFICE Document Server can be found in the project’s GitHub repository: `https://github.com/ONLYOFFICE/snap-documentserver/blob/master/snap/snapcraft.yaml <https://github.com/ONLYOFFICE/snap-documentserver/blob/d6ab8c34d3601d177b08c2ebaa68eb8fc98b8898/snap/snapcraft.yaml#L52>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/accounts_service.go

.. _`GNOME Online Accounts`: https://wiki.gnome.org/Projects/GnomeOnlineAccounts
.. _`ONLY OFFICE Document Server`: https://snapcraft.io/onlyoffice-ds
