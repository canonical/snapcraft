.. 6235.md

.. _how-to-use-the-system-gtk-theme-via-the-gtk-common-themes-snap:

How to use the system GTK theme via the gtk-common-themes snap
==============================================================

A common request is to `allow applications to follow the GTK theme of the host system desktop <https://snapcraft.io/docs/use-the-system-gtk-theme?u=jamesh>`__. While we don’t yet have all the infrastructure in place for arbitrary themes, we do now have one that will automatically handle the default themes of common Linux distributions. This will be sufficient for applications based on GTK 3, and Qt 5 apps using the ``libqgtk3.so`` platform theme.

I’ve put together a short Snapcraft project that demonstrates how everything fits together:

https://gist.github.com/jhenstridge/7f8c6d43bf21a42579e18d339be7a4fb

Now for a breakdown of the important parts of this project:

Content interface plugs
-----------------------

The theme data is made available through a set of content interface plugs. This can be achieved by adding the following boilerplate to the ``plugs:`` stanza of the project:

::

   plugs:
     gtk-3-themes:
       interface: content
       target: $SNAP/share/themes
       default-provider: gtk-common-themes
     icon-themes:
       interface: content
       target: $SNAP/share/icons
       default-provider: gtk-common-themes
     sound-themes:
       interface: content
       target: $SNAP/share/sounds
       default-provider: gtk-common-themes

This will cause various pieces of theme data to be mounted under ``$SNAP/share``, and to source it from the ``gtk-common-themes`` snap by default. As with other content interfaces, snapd will use the ``default-provider`` line to install the ``gtk-common-themes`` snap if it isn’t present.

In addition to the plug definitions, I’ve also made sure one of the parts will create the directories used as content interface mount points and ensure they are primed.

In the future, we hope to remove the need for much of this boilerplate through the use of `Snapcraft templates <https://snapcraft.io/docs/proposal-templates?u=jamesh>`__.

Allowing GTK to detect the selected system theme
------------------------------------------------

The way GTK determines what the current theme is depends on the windowing system in use:

-  On an X11 session, the `XSETTINGS specification <https://specifications.freedesktop.org/xsettings-spec/xsettings-latest.html>`__ is used. This only involves a set of X11 calls, so plugging the ``x11`` interface is sufficient.
-  On a Wayland session, GTK instead uses the ``GSettings`` API to read the users preferences directly, bypassing the display server. So plugging the ``gsettings`` interface is also necessary.

Putting that all together, most apps should plug at least the following:

::

     - desktop
     - gsettings
     - wayland
     - x11

How do I know if everything is working?
---------------------------------------

Other than changing your desktop’s current theme, GTK lets you use the ``GTK_THEME`` environment variable to override the default theme. So using our example project, I can run the app using a theme not currently installed on the host system:

::

   $ GTK_THEME=Arc-Dark gtk3-demo

You can also inspect the available themes within the confinement sandbox:

::

   $ snap run --shell gtk3-demo

   To run a command as administrator (user "root"), use "sudo <command>".
   See "man sudo_root" for details.

   $ ls $SNAP/share/themes
   Adwaita       Ambiance  Arc-Dark    Communitheme  EvoPop-Azure  Radiance
   Adwaita-dark  Arc   Arc-Darker  EvoPop    HighContrast

What about *un*\ common themes?
-------------------------------

We don’t currently have a system in place to automatically install third party theme snaps yet, but that should come in future. The way we’ve used the content interface is intended to allow third party themes to *supplement* those provided by ``gtk-common-themes`` rather than *replacing* those themes. So a GTK theme that re-uses a common icon theme doesn’t need to repackage all the icons.

If a third party snap also provides a ``gtk-3-themes`` slot, it can be connected to applications simultaneously with ``gtk-common-themes``. The ``gtk-common-themes`` snap can also be used as a template for your own third party theme snaps:

https://github.com/snapcrafters/gtk-common-themes

To test these additional snaps while we’re working on an automatic install solution, you will need to manually install the theme snap and then manually connect the interfaces to each application snap.
