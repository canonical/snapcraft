.. 7910.md

.. _the-removable-media-interface:

The removable-media interface
=============================

``removable-media`` allows read/write access to mounted removable storage in ``/media``, ``/run/media`` and ``/mnt``.

For example, this interface can be used to provide access to mounted USB sticks and external hard drives from the desktop environment.

Security-conscious users may want to remove sensitive external media when installing snaps, before disconnecting the interface from those snaps to prevent them from having access when the data is restored.

Access to ``/mnt`` requires snapd version *2.36+*.

--------------

**Auto-connect**: no

This interface is typically `not granted <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ the ability to :ref:`auto-connect <interface-management-auto-connections>` because doing so will permit access to potentially sensitive data such as photos, documents and encryption keys.

Certain application types may be granted an exception for the following reasons:

-  **major browsers**: typically designed with security and user privacy in mind
-  **media editors**: often used to import/edit/export large files on external devices
-  **media players/viewers**: often used to import/preview/playback files on external devices
-  **media recorders**: often used to export/edit files on external devices

**Transitional**: yes

``removable-media`` is considered a *transitional* interface because, when connected, only traditional UNIX permissions are considered for file access. This allows a background service in a snap to setup *inotify*, for example, to monitor, catalogue and modify files on external media.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
