.. 13485.md

.. _the-gnome-3-28-extension:

The gnome-3-28 extension
========================

This extension helps you snap desktop applications that use GTK 3, GNOME 3.28 and/or GLib.

How to use it
-------------

Add ``extensions: [ gnome-3-28 ]`` to the application definition in your ``snapcraft.yaml`` file. See :ref:`GTK3 applications <gtk3-applications>` for a complete tutorial on how to use this extension.

.. code:: yaml

   apps:
     foliate:
       command: usr/bin/com.github.johnfactotum.Foliate
       extensions: [gnome-3-28]
       ...

When to use it
--------------

Although this extensions adds support for the GTK 3 runtime, it also includes base desktop technologies such as GLib and cursor themes, so it is useful to almost any desktop application which does not have a more specialized extension available.

Some examples:

-  :ref:`GTK3 applications <gtk3-applications>`
-  :ref:`Java Swing applications <java-applications>`, except when they use :ref:`GTK+ 2 integration <gtk2-applications>`.
-  Games

This extension will *not* work for :ref:`GTK+ 2 applications <gtk2-applications>` and 32-bit applications.

See :ref:`Desktop Applications <desktop-applications>` for more information on how to snap a desktop application.

What it does
------------

-  It ensures the GTK3 and GNOME libraries are available to all parts at build and run time.
-  It initialises GTK3 and the desktop environment before your application starts so functionality like fonts, themes and a11y works correctly.

To do this, it connects each application to the following content snaps at run time.

-  `gtk-common-themes <https://snapcraft.io/gtk-common-themes>`__ for common GTK, icon, cursor and sound themes.
-  `gnome-3-28-1804 <https://snapcraft.io/gnome-3-28-1804>`__ for the GNOME runtime libraries and utilities corresponding to 3.28.

It also configures each application entry with these additional plugs:

-  :ref:`desktop <the-desktop-interface>`
-  :ref:`desktop-legacy <the-desktop-interface>`
-  :ref:`wayland <the-wayland-interface>`
-  :ref:`x11 <the-x11-interface>`

..

   ℹ Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.
