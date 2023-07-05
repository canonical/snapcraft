.. 7810.md

.. _the-docker-support-interface:

The docker-support interface
============================

``docker-support`` allows operating as the Docker daemon. This interface is for internal use by the `Docker snap <https://snapcraft.io/docker>`__. You should not need to use this interface in your snap.

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

**Attributes:**

* ``privileged-containers`` (plug): true|false (defaults to ``false``)

Can access resources and system calls necessary to run Docker application containers. The ``privileged-containers`` attribute may be used to give the necessary access to run privileged containers. Providing snaps specifying this interface currently may only be established with the Docker project.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
