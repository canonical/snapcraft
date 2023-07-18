.. 31449.md

.. _the-gnome-extension:

The gnome extension
===================

The *gnome* extension helps with the creation of snaps that use GTK 3, GNOME 42+, and/or GLib.

This is an updated version of the :ref:`gnome-3-38 extension <the-gnome-3-34-extension>` that will only work with the core22 :ref:`base snap <base-snaps>`, built from `Ubuntu 22.04 LTS <http://releases.ubuntu.com/22.04/>`__.

Previous Gnome extensions were built to use ``core18``, based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__ and ``core20``, based on `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__.

This extension provides many of the components needed for general desktop applications making it useful for a broader set of applications outside of those tailored for the GNOME desktop.

.. note::

   Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.


How to use it
-------------

This extension currently only works with the ``core22`` base snap (see :ref:`Base snaps <base-snaps>` for details). To use it, add ``extensions: [gnome]`` to the application definition in your :ref:`snapcraft.yaml <creating-snapcraft-yaml>` file.

For instance:

.. code:: yaml

   apps:
       tali:
           extensions: [gnome]
           command: usr/bin/tali
   [...]

See :ref:`GTK3 applications <gtk3-applications>` for a comprehensive overview of using extensions with GNOME applications.


Interface connections
---------------------

The following plugs are provided by the extension and implicitly included in your snapcraft.yaml:

.. code:: yaml

   plugs:
       desktop:
           mount-host-font-cache: false
       gtk-3-themes:
           interface: content
           target: $SNAP/data-dir/themes
           default-provider: gtk-common-themes
       icon-themes:
           interface: content
           target: $SNAP/data-dir/icons
           default-provider: gtk-common-themes
       sound-themes:
           interface: content
           target: $SNAP/data-dir/sounds
           default-provider: gtk-common-themes
       gnome-42-2204:
           interface: content
           target: $SNAP/gnome-platform
           default-provider: gnome-42-2204

Your app may still need additional plugs, but you can expect the following plugs to be automatically available to your apps as well:

::

   plugs: [ desktop, desktop-legacy, gsettings, opengl, wayland, x11, mount-observe, calendar-service]

See :ref:`Adding interfaces <adding-interfaces>` for more details.


Included packages
-----------------

The GNOME extension is derived from two separate snaps; a `build snap <https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204-sdk/snapcraft.yaml>`__ and a `platform snap <https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204/snapcraft.yaml>`__.

The **build snap** builds compiles libraries from source that are commonly used across GNOME applications. Examples include glib, gtk, and gnome-desktop. These are built to provide newer versions of these packages that exist in the core22 base snap (a subset of the Ubuntu 22.04 archive).

The **platform snap** takes the build snap and makes all of those libraries available to your snap at build time without needing to include the pieces of the build snap that are unnecessary at runtime (like compilers) in your final snap.


Environment variables
---------------------

In addition to using the build and platform snaps, the *gnome-3-38 extension* also sets a collection of environment variables, links, default plugs for the app to use, and a default build-environment for each part in your snap to use.

Build variables
---------------

The following “build-environment” section is made available to each part built in your snap.

If you define other build-environment variables, then those will get added to these and the set is used. If you define another value for one of these variables, then the value you’ve defined will be used instead of the value defined within the extension.

.. code:: yaml

   build-environment:
   -   PATH: /snap/gnome-42-2204-sdk/current/usr/bin${PATH:+:$PATH}
   -   XDG_DATA_DIRS: $SNAPCRAFT_STAGE/usr/share:/snap/gnome-42-2204-sdk/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}
   -   LD_LIBRARY_PATH: /snap/gnome-42-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET:/snap/gnome-42-2204-sdk/current/usr/lib:/snap/gnome-42-2204-sdk/current/usr/lib/vala-current:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
   -   PKG_CONFIG_PATH: /snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-42-2204-sdk/current/usr/lib/pkgconfig:/snap/gnome-42-2204-sdk/current/usr/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}
   -   GETTEXTDATADIRS: /snap/gnome-42-2204-sdk/current/usr/share/gettext-current${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}
   -   GDK_PIXBUF_MODULE_FILE: /snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache
   -   ACLOCAL_PATH: /snap/gnome-42-2204-sdk/current/usr/share/aclocal${ACLOCAL_PATH:+:$ACLOCAL_PATH}
   -   PYTHONPATH: /snap/gnome-42-2204-sdk/current/usr/lib/python3.10:/snap/gnome-42-2204-sdk/current/usr/lib/python3/dist-packages:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/gobject-introspection${PYTHONPATH:+:$PYTHONPATH}

Runtime variables
-----------------

The following environment is set when your application is run:

.. code:: yaml

   environment:
     SNAP_DESKTOP_RUNTIME: $SNAP/gnome-platform
     GTK_USE_PORTAL: '1'


Layouts set
-----------

The platform snap’s gjs, webkit2gtk-4.0, and iso-codes are used so they don’t need to be packaged as part of the snap and would greatly inflate the size.

.. code:: yaml

       /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/libgweather-4:
           symlink: $SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/libgweather-4
       /usr/lib/evolution-data-server:
           symlink: $SNAP/usr/lib/evolution-data-server
       /usr/bin/gnome-control-center:
           symlink: $SNAP/usr/bin/gnome-control-center
       /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0:
           bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0
       /usr/share/xml/iso-codes:
           bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
       /usr/share/libdrm:
           bind: $SNAP/gnome-platform/usr/share/libdrm

See :ref:`Snap layouts <snap-layouts>` for further details.
