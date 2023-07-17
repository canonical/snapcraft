.. 17331.md

.. _how-to-use-xdg-desktop-portals:

How to use XDG portals in your snap
===================================

:ref:`XDG portals <xdg-desktop-portals>` are used to define interaction between
a sandboxed application and external resources. A number of common use cases
for desktop applications are supported by portals.

Use the Portal APIs
-------------------

Unlike regular :ref:`Snapcraft interfaces <supported-interfaces>`, portals require applications to use a new API in order to access resources.

Toolkits like GTK 3 and Qt5, however, provide transparent support for portals. If your application is not using one of those toolkits, you will need to use
the Portals API directly. See the `Portals API documentation`_ for more
information.

Add the desktop interface to your snap
--------------------------------------

Adding the :ref:`desktop interface <the-desktop-interface>` to the snap gives
your snap access to the portals.

Enable portal support
---------------------

The way that portal support is enabled will depend on the technologies and
libraries that an application uses:

-  **GTK 3**: turn on portal support in GTK 3 by setting the following environment variable: ``GTK_USE_PORTAL=1``

-  **Qt**: often defaults to using portals, but you can enable it manually by changing the platform theme: Set ``QT_QPA_PLATFORMTHEME=gtk3`` on GTK based desktops and Set ``QT_QPA_PLATFORMTHEME=flatpak`` and ``QT_QPA_FLATPAK_PLATFORMTHEME=kde`` for Qt based desktops.

-  **Electron**: portal support in the Electron file chooser `is being worked on <https://github.com/electron/electron/pull/19159>`__.

..

   ⓘ Both the :ref:`gnome-3-34 extension <the-gnome-3-34-extension>` and the :ref:`kde-neon extension <the-kde-neon-extension>` automatically enable portal support for GTK 3 and Qt applications on GTK-based desktops. If your snap uses either extension, you only need to do step 1.

.. _`Portals API documentation`: https://flatpak.github.io/xdg-desktop-portal/portal-docs.html
