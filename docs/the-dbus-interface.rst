.. 2038.md

.. _the-dbus-interface:

The dbus interface
==================

``dbus`` enables ownership of a specific name on the session or system D-Bus.

**Auto-connect**: no

**Attributes**:

* ``name`` (slot): well-known D-Bus connection name for the service (eg, ``org.foo.bar``)
* ``bus`` (slot): D-Bus bus to use (ie, ``session`` or ``system``)
* ``name`` (plug): well-known D-Bus connection name of the service from the providing snap
* ``bus`` (plug): D-Bus bus to use for providing snap

Snaps that want to communicate via a well-known D-Bus connection name need to have matching ``bus`` and ``name`` attributes and then be connected via ``snap connect``. Snaps specifying ``bus: system`` will have a default D-Bus bus policy that allows ``root`` to own the name and anyone to send to a destination that matches the well-known name (eg, ``org.foo.bar``). Once connected, the consuming snap may communicate with the providing snap via:

-  the specified well-known D-Bus connection name (eg, ``org.foo.bar``)
-  a unique D-Bus connection name using a matching D-Bus interface (eg, ``org.foo.bar.baz``) or D-Bus path (eg, ``/org/foo/bar/norf``)

This interface is particularly well-suited for leaf-style applications from GNOME, KDE, etc to integrate into the desktop session. Future versions of snapd may allow greater flexibility for bus policy, D-Bus interfaces and D-Bus paths and also support session services and D-Bus activation.

Note: when developing snaps using devmode for D-Bus system services, the slot implementation must use this interface so that snapd may adjust the D-Bus bus policy so the snap may use the system bus.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

Using the D-Bus interface
-------------------------

The dbus interface provides a way for snaps to communicate over D-Bus. The snap providing the D-Bus service declares a slot with the well-known D-Bus name and which bus it uses. Snaps wanting to communicate with the providing snap’s service declare a plug for the providing snap. Note that a snap declaration is needed for your snap to be delivered via the snap store and claim this well-known D-Bus name (simply upload the snap to the store and request a manual review and a reviewer will take a look).

When a providing snap is installed, snapd will generate security policy that will allow it to listen on the well-known D-Bus name on the specified bus. If the ``system`` bus is specified, snapd will also generate D-Bus bus policy that allows ‘root’ to own the name and any user to communicate with the service. Non-snap processes are allowed to communicate with the providing snap following traditional permissions checks. Other (consuming) snaps may only communicate with the providing snap by connecting the snaps’ interface.

Examples
--------

Desktop application
~~~~~~~~~~~~~~~~~~~

Desktop applications sometimes listen on a well-known D-Bus name so that other desktop session services and applications may interact with them. This is common for GNOME and KDE applications, for example. Snaps listening on the session bus may only be accessed by processes running in the user’s session.

Providing snap (slot)
^^^^^^^^^^^^^^^^^^^^^

As a developer, if your application listens on the ``session`` bus with the well-known name “org.example.foo”, then you might adjust your snap’s yaml to have:

::

   name: foo
   ...
   slots:
     dbus-svc: # name that is used with 'snap connect' on slots side
       interface: dbus
       bus: session
       name: org.example.foo
   ...
   apps:
     foo:
       command: ...
       slots: [ dbus-svc ]

Consuming snap (plug)
^^^^^^^^^^^^^^^^^^^^^

As a developer, if you want your snap to communicate over D-Bus with another snap via the “org.example.foo” D-Bus well-known name on the ``session`` bus, you might adjust your snap’s yaml to have:

::

   name: bar
   ...
   plugs:
     foo-svc:  # name that is used with 'snap connect' on plugs side
       bus: session
       interface: dbus
       name: org.example.foo
   ...
   apps:
     bar:
       command: ...
       plugs:
       - foo-svc

System service
~~~~~~~~~~~~~~

Services may also listen on the D-Bus system bus. Snaps listening on the system bus run as root and may be accessed by any user on the system. While the snapd security policy will limit which connecting snaps can communicate with the service, slot implementations may want to perform additional permission checks (eg, UID) to further limit access.

Providing snap
^^^^^^^^^^^^^^

As a developer, if your application listens on the ``system`` bus with the well-known name “org.example.foo”, then the snap’s yaml is the same as above, but specify the ``system`` bus:

::

   name: foo
   ...
   slots:
     dbus-svc: # name that is used with 'snap connect' on slots side
       interface: dbus
       bus: system
       name: org.example.foo
   ...
   apps:
     foo:
       command: ...
       slots: [ dbus-svc ]

Consuming snap
^^^^^^^^^^^^^^

As a developer, if you want your snap to communicate over D-Bus with another snap via the “org.example.foo” D-Bus well-known name on the ``system`` bus, then the snap’s yaml is the same as above, but specify the ``system`` bus:slight_smile:

::

   name: bar
   ...
   plugs:
     foo-svc:  # name that is used with 'snap connect' on plugs side
       bus: system
       interface: dbus
       name: org.example.foo
   ...

Snap dbus interface connections
-------------------------------

As a user, if you want to allow ``bar`` to communicate with ``foo`` via D-Bus, then you can connect the interface like so:

.. code:: bash

   $ sudo snap connect bar:foo-svc foo:dbus-svc

You can check it’s worked with the following:

.. code:: bash

   $ snap connections
   Interface           Plug                 Slot               Notes
   dbus-svc            bar:dbus-svc         foo:dbus-svc       -
   [...]

To disconnect:

::

   $ sudo snap disconnect bar:foo-svc foo:dbus-svc

Future work
-----------

Autostarting of D-Bus session services and D-Bus activation is not supported at this time, but is planned. When that support is added, this document will be updated accordingly.

References
----------

-  https://github.com/snapcore/snapd/wiki/Interfaces#dbus
-  https://github.com/snapcore/snapd/blob/master/interfaces/builtin/dbus.go
