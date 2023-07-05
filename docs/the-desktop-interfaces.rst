.. 2042.md

.. _the-desktop-interfaces:

The desktop interfaces
======================

Common desktop interfaces
-------------------------

Several interfaces exist related to running desktop applications:

-  desktop (snapd 2.28): access to basic graphical resources (eg, GNOME, Plasma, etc)
-  desktop-legacy (snapd 2.28): access to legacy graphical resources (eg, a11y, ibus, fcitx, etc)
-  wayland (snapd 2.28): access to the wayland compositor
-  unity7: access to X, Unity services, a11y, input methods (ibus, fcitx, etc) and other legacy desktop methods
-  x11: access to the X server

The careful reader may notice overlap between some of these interfaces. Historically, snapd was going to add interfaces for each desktop environment as they were enabled in Snap, but later it was decided to `clean this up <https://snapcraft.io/docs/desktop-interfaces-moving-forward>`__. In addition to the cleanup, we wanted a path to transition away from the insecure X server and Desktop Environments with the outdated session trust model to the future where Wayland replaces X and desktop services are designed with the contemporary untrusted-app trust model. Essentially the cleanup consisted of:

-  grouping all contemporary Desktop Environment default policy into a new ``desktop`` interface. Contemporary DE’s are defined to include those that can run without X (and with wayland or mir) with the security policy allowing access to common, safe services
-  grouping security policy for unsafe services into the new ``desktop-legacy`` interface. As the Linux desktop matures and the unsafe services are replaced or made safe, the new safe accesses will be added to the ``desktop`` interface
-  the unity7 interface remains for compatibility for typical, non-Wayland capable DEs
-  the x11 and wayland (and mir) interfaces are broken out from the ``desktop`` interface so developers may specify the environments where the application is expected to work

Importantly, with the above changes, existing snaps continue to work. Developers wanting to work with newer DEs add these newer interfaces to their ``plugs`` as their apps support them.

Other desktop interfaces
------------------------

Many other optional interfaces exist depending on what the app needs:

-  :ref:`audio-playback <the-audio-playback-interface>` and :ref:`audio-record <the-audio-record-interface>`: play and/or record audio
-  :ref:`browser-support <the-browser-support-interface>`: access required for browsers and applications built on top of browser technologies (eg, electron)
-  :ref:`cups-control <the-cups-control-interface>`: for printing via CUPS directly
-  :ref:`dbus <the-dbus-interface>`: allow access to app via a well-known DBus connection name
-  :ref:`gsettings <the-gsettings-interface>`: read/write access to global session settings
-  :ref:`home <the-home-interface>`: read/write access to non-hidden files in $HOME
-  :ref:`mpris <the-mpris-interface>`: allow other processes to use this snap’s MPRIS DBus API to control media playback
-  :ref:`opengl <the-opengl-interface>`: direct access of OpenGL and EGL devices
-  :ref:`password-manager-service <the-password-manager-service-interface>`: read/write access to saved passwords
-  :ref:`removable-media <the-removable-media-interface>`: access files on removable media
-  :ref:`screen-inhibit-control <the-screen-inhibit-control-interface>`: prevent screen sleep/lock

While developers might choose to use any of the above interfaces as needed, it should be noted that many are so-called ‘transitional’ interfaces. For example, ``gsettings`` allows read/write access to all settings and ``home`` allows read/write access to all non-hidden user data. Like with the common services in ``desktop-legacy``, as these other services are made safe or new ones designed to replace them, they will be added to the ``desktop`` interface. :ref:`xdg-desktop-portals`, for example provide safe APIs for opening files and printing.

Example usage for common desktop interfaces
-------------------------------------------

Traditional desktop app (desktop snaps before snapd 2.28)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As a developer, before snapd 2.28, the snap’s yaml for a typical desktop application might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - unity7

On systems with snapd 2.28+, existing and new apps can continue to only plugs ``unity7`` and be expected to work everywhere they would on systems with 2.27 or earlier.

Wayland-only desktop app
~~~~~~~~~~~~~~~~~~~~~~~~

As a developer, the snap’s yaml for a desktop app that only works with GNOME Shell/Plasma and Wayland might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland

Wayland desktop app with X fallback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As a developer, the snap’s yaml for a desktop app that only works with GNOME Shell/Plasma with either X or Wayland might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland
       - x11

Desktop app that can run anywhere
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

While the Linux desktop is transitioning away from X-based toolkits to ones built on top of Wayland compositors, many application developers may want to choose all the DEs where the application may run. Newer GTK, Qt and other toolkits are capable of running on systems that run X or Wayland or in different DEs like GNOME Shell, Plasma or Unity.

Therefore, as a developer, the snap’s yaml for a desktop app that works on a wide range of DEs might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland
       - unity7

Example usage with other interfaces
-----------------------------------

GNOME-based editor
~~~~~~~~~~~~~~~~~~

As a developer, the snap’s yaml for a GNOME-based editor that works on a wide range of DEs, needs access for printing and needs access to files in the user’s home directory might include:

.. code:: yaml

   name: foo
   ...
   slots:
     foo-svc:
       interface: dbus
       bus: session
       name: org.gnome.foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland
       - unity7
       - cups-control
       - gsettings
       - home
       slots:
       - foo-svc

Media player
~~~~~~~~~~~~

As a developer, the snap’s yaml for a desktop app that plays audio and video, works on a wide range of DEs, needs access to files in the user’s home directory, uses the network for fetching media art and can be controlled via MPRIS might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland
       - unity7
       - home
       - network
       - opengl
       - audio-playback
       slots:
       - mpris

Electron app
~~~~~~~~~~~~

As a developer, the snap’s yaml for an Electron desktop app that works on a wide range of DEs might include:

.. code:: yaml

   name: foo
   ...
   apps:
     foo:
       plugs:
       - desktop
       - desktop-legacy
       - wayland
       - unity7
       - alsa
       - avahi-observe
       - browser-support
       - camera
       - cups-control
       - gsettings
       - home
       - network
       - opengl
       - audio-playback
       - screen-inhibit-control
       - upower-observe

For the interfaces listed above that were not already discussed, see https://github.com/snapcore/snapd/wiki/Interfaces.

Extra information
-----------------

Graphical applications also require additional libraries and environment configuration to function correctly inside a snap. Snapcraft has various tools to help you with that. Read the :ref:`snapping desktop applications <desktop-applications>` documentation for more information about these tools. Please feel free to ask questions in the `forum <https://forum.snapcraft.io>`__ or on `Rocketchat <https://rocket.ubuntu.com/channel/snapcraft>`__ if you are having trouble.

The ``snappy-debug`` tool can help identify interfaces your snap needs. See the `forum <https://snapcraft.io/docs/security-policy-and-sandboxing>`__ for details.

References
----------

-  :ref:`Supported Interfaces <supported-interfaces>`
-  https://snapcraft.io/docs/desktop-interfaces-moving-forward
-  https://github.com/ubuntu/snapcraft-desktop-helpers
