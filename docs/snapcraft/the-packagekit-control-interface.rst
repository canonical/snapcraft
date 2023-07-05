.. 13052.md

.. _the-packagekit-control-interface:

The packagekit-control interface
================================

``packagekit-control`` allows control of the `PackageKit <https://www.freedesktop.org/software/PackageKit/>`__ service, giving privileged access to native package management on the system.

This interface is intended to work in tandem with :ref:`the AppStream interface <the-appstream-metadata-interface>`. Snaps distributed via the public `Snap store <https://snapcraft.io/store>`__ are not typically granted auto-connection for this interface.

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

Requires snapd version *2.41+*.\`

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
