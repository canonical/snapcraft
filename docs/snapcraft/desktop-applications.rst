.. 13034.md

.. _desktop-applications:

Desktop applications
====================

Distributing a Desktop GUI application for Linux while reaching the widest possible audience is complicated. Typically, the user has to make sure the correct version of the GUI toolkit is installed and configured. When a Linux distribution changes the delivered GUI toolkits, this can be problematic for applications.

Snaps solve these problems and ensure the correct toolkit libraries are shipped alongside the application at all times.

Getting Started
---------------

Read the documentation of the GUI toolkit your application uses in order to get started!

+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| Common GUI Toolkits                                                  | Technologies                                                                                             |
+======================================================================+==========================================================================================================+
| :ref:`GTK 3 <gtk3-applications>`                                     | :ref:`OpenGL/GPU support <adding-opengl-gpu-support-to-a-snap>`                                          |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| :ref:`GTK+ 2 <gtk2-applications>`                                    | :ref:`Desktop Launchers / Menu entries <desktop-files-for-menu-integration>`                             |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| :ref:`Qt 5 and KDE Frameworks <qt5-and-kde-frameworks-applications>` | :ref:`XDG desktop portals <xdg-desktop-portals>`                                                         |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| :ref:`Java Swing <java-applications>`                                | :ref:`AppStream metadata support <using-external-metadata>`                                              |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| :ref:`Electron <electron-apps>`                                      | :ref:`chromium-ffmpeg in third-party browser snaps <using-chromium-ffmpeg-in-third-party-browser-snaps>` |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+
| :ref:`Flutter <flutter-applications>`                                | :ref:`The desktop interfaces <the-desktop-interfaces>`                                                   |
+----------------------------------------------------------------------+----------------------------------------------------------------------------------------------------------+

Refining
--------

-  :ref:`Reduce the size of your snap <reducing-the-size-of-desktop-snaps>`. This will also speed up how quickly your snap starts!
-  `Switch the compression to ``lzo`` <snapcraft-top-level-metadata-compression>`__ to make your application start up even quicker.
-  Make sure your application has a :ref:`logo in the snap store <store-listing-and-branding-logo-icon>` and :ref:`screenshots <store-listing-and-branding-screenshots>`.
-  Include a link to your contact page or bugtracker in the :ref:`store metadata <store-listing-and-branding-metadata>`.

Further Information
-------------------

Compared to CLI apps, desktop apps typically require three additional features.

1. Access to the host system to play sound, create notifications, display a window, etc.
2. Access to a GUI toolkit such as GTK or Qt.
3. Initialisation of desktop-specific functionality such as fonts, themes and the `XDG <https://www.freedesktop.org>`__ environment.

Since snap apps are completely sandboxed by default, they cannot play sound, create notifications, or access the X server to display itself. However, it’s easy to make this possible by using the :ref:`desktop interfaces <the-desktop-interfaces>`. These allow you to “poke holes” in the sandbox, to give your application selected access to the host system.

The sandbox also means that your app cannot use the GUI toolkits of the host system. The snap either has to include the toolkit in the snap itself, or it needs to connect to a snap that provides these toolkits. Many toolkits and general desktop features such as fonts and themes also require initialization before the application can start. The extensions make this as easy as possible. They provide parts to bundle or access common GUI toolkits in your snap and a ``desktop-launch`` script which does the required initialization for you.

Legacy
------

These methods are not recommended anymore but might be useful as reference.

-  :ref:`Qt 5 support using the desktop-helpers <deprecated-desktop-app-support-qt5>`
-  `snapcraft-desktop-helpers <https://github.com/ubuntu/snapcraft-desktop-helpers/>`__ provided useful parts and launchers for desktop snaps, but these are deprecated in favor of the ``gnome-*`` and ``kde-neon`` extensions.


.. toctree::
   :hidden:

   desktop-app-support-gtk
   desktop-app-support-qt
   desktop-app-support-qt-for-snaps-without-bases
   desktop-app-support-qt4
   deprecated-desktop-app-support-qt5
   how-to-use-the-system-gtk-theme-via-the-gtk-common-themes-snap
