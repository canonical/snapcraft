.. 13752.md

.. _the-kde-neon-extension:

The kde-neon extension
======================

This extension helps you snap desktop applications that use Qt5 and/or `KDE Frameworks <https://kde.org/products/frameworks/>`__.

How to use it
-------------

Add ``extensions: [ kde-neon ]`` to the application definition in your ``snapcraft.yaml`` file. See :ref:`QT5 and KDE Frameworks applications <qt5-and-kde-frameworks-applications>` for a complete tutorial on how to use this extension.

.. code:: yaml

   apps:
     kcalc:
       extensions:
         - kde-neon
       command: kcalc
       ...

..

   ℹ If you are using ``core18`` as a base and your application needs access to the Qt5 and KDE Frameworks development tools, add `kde-frameworks-5-core18-sdk <https://snapcraft.io/kde-frameworks-5-core18-sdk>`__ to the ``build-snaps`` of the part that builds your application (this action is not required when using ``core20`` as a base).

What it does
------------

-  It makes the latest Qt5 and KDE Frameworks libraries available to your application at run time.
-  It initialises Qt5 and the desktop environment before your application starts so functionality like fonts, cursor themes and a11y work correctly.

To do this, it connects each application to the following content snaps at run time.

-  `gtk-common-themes <https://snapcraft.io/gtk-common-themes>`__ for common icon, cursor and sound themes.
-  `kde-frameworks-5-core18 <https://snapcraft.io/kde-frameworks-5-core18>`__ for the Qt5 and KDE Frameworks runtime libraries when the base is ``core18``.
-  `kde-frameworks-5-99-qt-5-15-7-core20 <https://snapcraft.io/kde-frameworks-5-99-qt-5-15-7-core20>`__ for the Qt5 and KDE Frameworks runtime libraries when the base is ``core20``.
-  `kde-frameworks-5-102-qt-5-15-8-core22 <https://snapcraft.io/kde-frameworks-5-102-qt-5-15-8-core22>`__ for the Qt5 and KDE Frameworks runtime libraries when the base is ``core22``.

It also configures each application entry with these additional plugs.

-  :ref:`desktop <the-desktop-interface>`
-  :ref:`desktop-legacy <the-desktop-interface>`
-  :ref:`opengl <the-opengl-interface>`
-  :ref:`wayland <the-wayland-interface>`
-  :ref:`x11 <the-x11-interface>`

For a complete picture of what this extension does, add it to your app definition and run ``snapcraft expand-extensions``.

Limitation
----------

Currently, this extension only supports snaps *targeting the amd64 architecture*.

   ℹ Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.
