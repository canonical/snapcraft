.. 6154.md

.. _interface-management:

Interface management
====================

Interfaces allow (or deny) access to a resource outside of a snap’s confinement.

Most users don’t need to worry about interfaces. Snaps are designed for strong application isolation and safe interface connections are made automatically.

An interface is most commonly used to enable a snap to access OpenGL acceleration, sound playback or recording, your network, and your $HOME directory. But which interfaces a snap requires, and *provides*, is very much dependent on the type of snap and its own requirements.

See :ref:`Supported interfaces <supported-interfaces>` for a comprehensive list of interfaces and what kind of access they permit.


.. _interface-management-snap-store:

Using a GUI
-----------

The Ubuntu Software/`Snap Store <https://snapcraft.io/snap-store>`__ desktop application is installed by default on Ubuntu and can be used to list an application’s interfaces and to connect and disconnect them.

An application first needs to be installed as a snap:

.. figure:: https://assets.ubuntu.com/v1/8905c627-store-01.png
   :alt: Snap Store VLC install from snap


To access the interface management functions, either search for an installed snap, or select it from the *Installed* view. The interfaces for the selected application can then be viewed by selecting **Permissions**:

.. figure:: https://assets.ubuntu.com/v1/7fbcf74c-store-04.png
   :alt: Snap store permissions


Each interface can now be connected or disconnected by selecting the toggle switch to the right of its description, and you may be prompted for your password.


.. _interface-management-listing:

Listing interfaces
------------------

On the terminal, the *snap* command provides more granular control over interface connections, and which interfaces are operational, on your system.

The ``snap connections`` command lists which interfaces are connected and being used, while adding ``--all`` additionally shows interfaces with unconnected slots or plugs (shown in the output as a ``-``):

.. code:: bash

   $ snap connections --all
   Interface            Plug                           Slot                     Notes
   adb-support          scrcpy:adb-support             :adb-support             -
   alsa                 ffmpeg:alsa                    :alsa                    manual
   appstream-metadata   snap-store:appstream-metadata  :appstream-metadata      -
   iaudio-playback      ardour:audio-playback          :audio-playback          -
   dbus                 -                              cameractrls:dbus-daemon  -
   [...]


.. _interface-management-listing-connections:

Listing connections
-------------------

To see which interfaces a snap is using, and which interfaces it could use but isn’t, type ``snap connections <snapname>``:

.. code:: bash

   $ snap connections vlc
   Interface       Plug                   Slot                 Notes
   audio-playback  vlc:audio-playback     :audio-playback      -
   audio-record    vlc:audio-record       -                    -
   camera          vlc:camera             -                    -
   desktop         vlc:desktop            :desktop             -
   home            vlc:home               :home                -
   (...)

In the above output, the :ref:`camera <the-home-interface>` interface is not connected because its slot is empty. This means VLC cannot access any connected cameras.

VLC can access the user’s */home* directory because the :ref:`home <the-home-interface>` interface is connected to the system ``$HOME`` directory (denoted by the ``:home`` slot name).

To see all connected interfaces on your system, use the *snap connections* command without a snap name:

.. code:: bash

   $ snap connections
   Interface      Plug                    Slot                 Notes
   adb-support    scrcpy:adb-support      :adb-support         -
   alsa           ffmpeg:alsa             :alsa                manual
   alsa           telegram-desktop:alsa   :alsa                manual
   audio-playback ardour:audio-playback   :audio-playback      -
   audio-playback chromium:audio-playback :audio-playback      -
   (...)

Adding ``--all`` to the *snap connections* command will list all interfaces, including those without a connection:

.. code:: bash

   $ snap connections --all
   Interface      Plug                    Slot                 Notes
   adb-support    scrcpy:adb-support      :adb-support         -
   alsa           entropypianotuner:alsa  -                    -
   alsa           ffmpeg:alsa             :alsa                manual
   alsa           guvcview:alsa           -                    -
   (...)


.. _interface-management-slots-plugs:

Plugs and slots
---------------

An interface provides a level of access to resources, such as audio playback, as defined by a *slot*. One or more snaps can access this resource by connecting a corresponding *plug* to the slot.

In other words, the slot is the provider of the resource while the plug is the consumer, and a slot can support multiple plug connections.

.. figure:: https://assets.ubuntu.com/v1/59c290a8-snapd-interfaces.png
   :alt: How an interfaces uses a plug and a slot


In the output to ``snap connections vlc`` (see above), every interface used by VLC is listed in the first column. The *Plug* and *Slot* columns then describe how each interface is connected.

For instance, the ``audio-playback`` interface connects VLC’s audio-playback plug to the system’s audio-playback slot so you can hear the sound it produces.

You can see which other snaps are using an interface with the ``interface`` command:

.. code:: bash

   $ snap interface audio-playback
   name:    audio-playback
   summary: allows audio playback via supporting services
   plugs:
     - chromium
     - vlc
     - zoom-client
   slots:
     - snapd

In the above output, you can see that Chromium, VLC and the Zoom snaps are connected to *snapd’s* audio-playback slot, which is synonymous with *Core* and *system*.


.. _interface-management-auto-connections:

Auto-connections
----------------

Many interfaces are automatically connected when a snap is installed, and this ability is a property of either the interface itself, or the snap.

Automatically connecting interfaces include the :ref:`network <the-network-interface>`, :ref:`audio-playback <the-audio-playback-interface>` and :ref:`opengl <the-opengl-interface>` interfaces. This *auto-connection* ability is carefully reviewed for each interface, where permissiveness, security and privacy implications, and the expectations of the user, are all considered.

A snap’s developer can also request that an interface is connected automatically through a :ref:`manual review process <permission-requests>`. As above, these requests are carefully considered and reviewed before being granted or denied.

Interfaces not connected automatically require the user to make a manual connection (see below), such as the :ref:`camera <the-camera-interface>`, :ref:`removable-media <the-removable-media-interface>` and :ref:`audio-record <the-audio-record-interface>` interfaces. Manual connections enable the user to have a complete control over what kind of access they allow.

If a snap is installed prior to an interface being granted auto-connect permission, and permission is subsequently granted and the snap updated, when the installed snap updates, the interface will be auto-connected.

For more technical details on how interface auto-connections are processed, see :ref:`The interface auto-connection mechanism <the-interface-auto-connection-mechanism>`.

   ⓘ See the *Auto-connect* column in the :ref:`Supported interfaces <supported-interfaces>` table for which interfaces are connected automatically.


.. _interface-management-manual-connections:

Manual connections
------------------

When you need to connect an interface manually, such as when you want to grant a snap access to :ref:`audio-record <the-audio-record-interface>` for audio input, use the ``snap connect`` command:

.. code:: bash

   snap connect <snap>:<plug interface>

With no further arguments, the plug will connect to the system via the snap daemon, *snapd*.

For example, to connect VLC’s *audio-record* plug to the system’s *audio-record*, you’d enter the following:

.. code:: bash

   sudo snap connect vlc:audio-record

To connect an interface to a slot provided by another snap, provide this as an additional argument:

.. code:: bash

   snap connect <snap>:<plug interface> <snap>:<slot interface>

A slot and a plug can only be connected if they have the same interface name.

Add the ``--no-wait`` option to *snap connect* or *snap disconnect* to run the process in the background and return immediately to the command prompt.

.. note::
          A successful connection grants any necessary permissions that may be required by the interface to function.


.. _interface-management-disconnect:

Disconnect interfaces
---------------------

To disconnect an interface, use ``snap disconnect``:

.. code:: bash

   snap disconnect <snap>:<plug interface>

Following our previous example, you would disconnect *vlc:audio-record* with the following command:

.. code:: bash

   sudo snap disconnect vlc:audio-record

When an automatic connection (`see above <interface-management-auto-connections_>`__) is manually disconnected, its disconnected state is retained after a `snap refresh <https://snapcraft.io/docs/managing-updates>`__. The ``--forget`` flag can be added to the disconnect command to reset this behaviour, and consequently, re-enable the automatic re-connection after a snap refresh.
