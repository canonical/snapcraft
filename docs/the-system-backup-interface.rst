.. 14348.md

.. _the-system-backup-interface:

The system-backup interface
===========================

``system-backup`` provides read-only access to the system via ``/var/lib/snapd/hostfs``. This interface gives privileged access to system data.

**Auto-connect**: no

**Transitional**: no

Requires snapd version *2.43+*.

By design, snaps using ``strict`` and ``devmode`` :ref:`confinement <snap-confinement>` run with a different root filesystem than the host. Snaps that implement a backup function, however, need access to the host’s snap-related files outside of this confinement, and this is where the ``system-backup`` interface can help.

The system-backup interface provides read-only access to files under the following location:

-  ``/var/lib/snapd/hostfs``

With the following exceptions:

- ``/var/lib/snapd/hostfs/dev``
- ``/var/lib/snapd/hostfs/proc``
- ``/var/lib/snapd/hostfs/sys``

Importantly, neither snapd nor this interface provide any mechanisms to ensure the system is in a ready state for backups. Reliable backup strategies with snaps utilising this interface will need to account for this themselves.

   ⓘ This is a snap interface. See :ref:`interface-management` and :ref:`supported-interfaces` for further details on how interfaces are used.
