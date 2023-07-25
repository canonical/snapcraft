.. 11703.md

.. _deprecated-desktop-app-support-qt5:

[Deprecated] Desktop App support - Qt5
======================================

   ⚠ **This tutorial is outdated.** See :ref:`Qt 5 and KDE Frameworks applications <qt5-and-kde-frameworks-applications>` for a much easier way to snap Qt 5 applications.

   This tutorial should only be used when you want to support architectures other than amd64, or for classic snaps.

This guide explains the recommended way to make a qt5 application function correctly inside a snap. **These instructions need to be applied in addition to the instructions for**\ :ref:`creating a snap <creating-a-snap>`\ **in your programming language.** For example, if your application uses the C programming language and Qt5, you also need to follow the instructions for :ref:`snapping a C/C++ application <c-c-applications>`.

The `Snapcraft Desktop Helpers project <https://github.com/ubuntu/snapcraft-desktop-helpers>`__ provides the ``desktop-qt5`` part to include the Qt5 dependencies in your snap. It also includes the ``desktop-launch`` script to initialise the desktop-specific functionality before your application launches.

1. Add qt5 dependencies to your snap
------------------------------------

Copy the definition of the ``desktop-qt5`` part from the `Snapcraft Desktop Helpers <https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml>`__ :file:`snapcraft.yaml` file into your Snapcraft recipe, in the ``parts`` section. Change the ``source`` property of this part to ``https://github.com/ubuntu/snapcraft-desktop-helpers.git``.

.. code:: yaml

     # This part installs the qt5 dependencies and a `desktop-launch` script to initialise
     # desktop-specific features such as fonts, themes and the XDG environment.
     #
     # It is copied straight from the snapcraft desktop helpers project. Please periodically
     # check the source for updates and copy the changes.
     #    https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml
     #
     desktop-qt5:
       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
       source-subdir: qt
       ...
       <copy the rest of this part from the desktop helpers snapcraft.yaml>
       ...

..

   **NOTE:** In order to keep the integration up-to-date for the best user experience, you should check if the definition in the Snapcraft desktop helpers repo has been changed periodically.

2. Initialise desktop-specific functionality
--------------------------------------------

Insert ``bin/desktop-launch`` into the app’s command chain.

.. code:: yaml

   apps:
     qt5app:
       command: bin/desktop-launch qt5app

If you’re using the ``full`` adapter, insert the ``bin/desktop-launch`` launcher into the list of the ``command-chain`` key of the ``apps._app_name_`` property instead:

.. code:: yaml

   apps:
     qt5app:
       adapter: full
       command: bin/qt5app
       command-chain:
       - bin/desktop-launch

..

   **NOTE:** If your application requires recent Qt or KDE libraries that aren’t available from Ubuntu LTS, the ``kde-frameworks`` content and SDK snaps might be useful. Please take a look at `KDE Snap Guide`_ to see how to use this SDK. This SDK only supports the AMD64 processor architecture.

3. [optional] Add GTK theme integration
---------------------------------------

Snapped Qt applications by default don’t use the UI theme of the operating system they are running on. You can make your snapped application adapt to most common UI themes using the ``gtk-common-themes`` content snap and the ``gtk3`` Qt Platform theme.

Add the following ``qt5-gtk-platform`` part into the ``parts`` section of your :file:`snapcraft.yaml` file.

.. code:: yaml

     qt5-gtk-platform:
       plugin: nil
       stage-packages:
         - qt5-gtk-platformtheme

In the ``apps._app_name_`` stanza, merge the following ``environment`` property:

.. code:: yaml

   apps:
     qt5app:
       environment:
         # Use GTK3 cursor theme, icon theme and open/save file dialogs.
         QT_QPA_PLATFORMTHEME: gtk3

In the ``plugs`` stanza, add the following plug definition:

.. code:: yaml

   plugs:
     # Support for common GTK themes
     # how-to-use-the-system-gtk-theme-via-the-gtk-common-themes-snap.md
     gsettings:
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

Now your applications will use the icon theme, cursor theme and open/save file dialogs of the desktop it runs on. The colors of your applications will also adapt to the GTK theme colors of the operating system.

Examples
--------

You can take a look at the `snapcraft.yaml file of keepassxc <https://github.com/keepassxreboot/keepassxc/blob/develop/snap/snapcraft.yaml>`__ for a complete example of a qt5 snap.

References
----------

-  https://snapcraft.io/docs/qt-apps-and-gtk-themes-an-investigation-with-partial-success15

.. _`KDE Snap Guide`: https://community.kde.org/Guidelines_and_HOWTOs/Snap
