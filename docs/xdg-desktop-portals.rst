.. 17331.md

.. _xdg-desktop-portals:

XDG desktop portals
===================

Cross Desktop Group (XDG) portals are a standardised framework allowing desktop applications to use resources outside of their sandbox. The file chooser portal, for example, opens a native file chooser on the host system. When the user selects a file, the application is granted access to that file.

Portals provide a range of common features to applications, including:

-  Opening a file with a file chooser
-  Opening URIs in other applications
-  Preventing the device from suspend/sleep/powering off
-  Sending email
-  Showing notifications
-  Taking screenshots and screencasts

See the `Portal API reference`_ for all supported portals.

   ⓘ Portals originated from the `Flatpak`_ project, but are now a common Linux desktop standard with support from GNOME, KDE and Snapcraft. They are even used outside of sandboxes to provide a standardised API to common desktop features such as `screenshots and screen casts on wayland <https://github.com/emersion/xdg-desktop-portal-wlr/wiki/FAQ>`__.


File chooser portal vs home interface
-------------------------------------

It is recommended to use the file chooser portal instead of the :ref:`home <the-home-interface>` and :ref:`removable-media <the-removable-media-interface>` interfaces for the following reasons:

-  The portal gives the user complete control over what exact files your application should access while the interfaces are all-or-nothing toggles.
-  The portal works with hidden files and folders in the home directory. If a user chooses a hidden file, the portal will give your application access to it. The ``home`` interface does not give your app access to hidden files and folders in the home directory for security reasons. Note that the ``home`` interface does give access to hidden files and folders elsewhere, just not in the home directory itself.
-  The portal works with removable-media out of the box. If a user chooses a file from a USB stick, your app will get access to it. The ``removable-media`` interface, however, does not auto-connect by default.

However, the file chooser portal works a bit differently than the home interface:

-  Files are fuse-mounted to ``/run/user/<uid>/doc/<hash>/`` in order to give your application access to it. So the path your application sees is different from the path a user chose, even though both are the same file.

The FileChooser portal also contains a few bugs:

-  `“Executable” permissions are currently not retained <https://github.com/flatpak/xdg-desktop-portal/issues/517>`__. All files will appear as non-executable to your application.
-  Support for selecting folders instead of files `has recently been merged <https://github.com/flatpak/xdg-desktop-portal/pull/456>`__ and is not released yet. This will only work on distributions using ``xdg-desktop-portal`` > 1.8.0.
-  Currently, the files are mounted even if your application has access to the file using another interface. See `Improving XDG Desktop Support <https://snapcraft.io/docs/improving-xdg-desktop-portal-support>`__ for the current status on fixing this issue.

Known Limitations
-----------------

-  ``org.freedesktop.portal.Flatpak.Spawn`` only works in a Flatpak. If your application needs to run arbitrary binaries on the host system, you can use :ref:`classic confinement <snap-confinement>`.
-  Portal support depends on the version of ``xdg-desktop-portal`` in the host system. Older versions do not support all portals. `Repology <https://repology.org/project/xdg-desktop-portal/versions>`__ shows what version of ``xdg-desktop-portal`` each distribution has and the `portals NEWS <https://github.com/flatpak/xdg-desktop-portal/blob/master/NEWS>`__ file explains what portals each version supports.

..

   ⓘ See :ref:`Desktop applications <desktop-applications>` for more information on how to snap a desktop application.

.. _`Portal support in GTK 3`: https://docs.flatpak.org/en/latest/portals-gtk.html
.. _`Portal support in Qt5 and KDE`: https://docs.flatpak.org/en/latest/portals-qt.html
.. _`Portal API reference`: https://flatpak.github.io/xdg-desktop-portal/portal-docs.html
.. _`Flatpak`: https://flatpak.github.io/
