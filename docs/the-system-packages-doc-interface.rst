.. 17788.md

.. _the-system-packages-doc-interface:

The system-packages-doc interface
=================================

``system-packages-doc`` interface allows read-only access to ``/usr/share/doc`` on the host system, a directory that typically contains system documentation.

After the interface has been connected, the host’s ``/usr/share/doc`` directory replaces the ``/usr/share/doc`` for the context of the snap. This could potentially hide existing in-snap content from the application.

This interface is helpful for *web browsers*, for example, because it enables them to open and view the host’s HTML system documentation.

The interface it is not connected automatically because the listed documentation can be used to infer which packages are installed on the host system.

**Auto-connect**: no

Requires snapd version *2.46+*.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
