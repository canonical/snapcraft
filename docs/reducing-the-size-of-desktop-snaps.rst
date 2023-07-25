.. 17280.md

.. _reducing-the-size-of-desktop-snaps:

Reducing the size of desktop snaps
==================================

One way to improve the user experience of your snaps is by reducing their size. Not only does this reduce the amount of **storage and network bandwidth** required to install and update your snap, but it also reduces its **startup time**. Since snaps are stored compressed on disk, they need to be decompressed when your application starts, and that takes longer when your snap is larger.

Using content snaps and extensions
----------------------------------

Snaps can provide content to other snaps using the :ref:`content interface <the-content-interface>`. Many popular runtimes and libraries such as GNOME, Qt and Wine are packaged in content snaps. Instead of adding those libraries to your snap, you can connect to the relevant content snaps. The easiest way to use content snaps is to use one of the :ref:`Snapcraft desktop extensions <snapcraft-extensions>`.

-  Snaps using KDE Frameworks or Qt5 can use the :ref:`kde-neon extension <the-kde-neon-extension>` which adds content snaps for themes, the Qt5 runtime and KDE frameworks.
-  Most other desktop snaps can use the :ref:`gnome-3-28 extension <the-gnome-3-28-extension>`.
-  Snaps using Wine can use the `wine-platform-runtime <https://snapcraft.io/wine-platform-runtime>`__ and `wine-platform-5-stable <https://snapcraft.io/wine-platform-5-stable>`__ snaps.
-  The :ref:`gtk-common-themes <how-to-use-the-system-gtk-theme-via-the-gtk-common-themes-snap>` bundles many popular GTK, cursor and icon themes.

..

   ⚠ Updating an existing snap to use a content snap or extension will not automatically reduce the size of your snap. You also need to manually remove the relevant libraries from your snap by removing them from ``stage-packages``, for example. The :ref:`cleanup part <reducing-size-cleanup-part>` can help you do this automatically.

Avoid staging development files
-------------------------------

The Ubuntu archives include many packages with development files whose name ends in ``-dev``. These packages are required to compile software which uses the libraries. For example, in order to compile software which uses ``libgtk-3-0`` (10.1 MB), you also need to install ``libgtk-3-dev`` (+14.0 MB). After compilation, these packages are not needed anymore, so it’s important to make sure they are not included in your snap. In the example of ``libgtk-3-0``, this will more than halve the size of the library in your snap.

To avoid this, move all ``stage-packages`` which end in ``-dev`` to ``build-packages`` and add their regular counterpart to ``stage-packages``. See `Build and staging dependencies <https://snapcraft.io/docs/build-and-staging-dependencies>`__ for more information.


.. _reducing-size-cleanup-part:

Using the cleanup part
----------------------

Snaps often ship libraries that are already available in the base snap or connected content snaps. This is because ``stage-packages`` currently pulls in all the dependencies of the packages you list, even if those dependencies are already available from content snaps.

You can remove those libraries manually by excluding them using the :ref:`snapcraft-yaml-reference-stage` or :ref:`snapcraft-yaml-reference-prime` keywords. This requires a lot of manual work, however, because you have to figure out which libraries are duplicated and exclude each one manually. An easier method is to use the following ``cleanup`` part, which automatically compares your snap with all the content snaps you specify and removes duplicate files.

1. Add the following part to your :file:`snapcraft.yaml` file in the ``parts`` section.

   .. code:: yaml

      # This part removes all the files in this snap which already exist in
      # connected content and base snaps. Since these files will be available
      # at runtime from the content and base snaps, they do not need to be
      # included in this snap itself.
      #
      cleanup:
        after:  # Make this part run last; list all your other parts here
          - <YOUR-PARTS>
        plugin: nil
        build-snaps:  # List all content-snaps and base snaps you're using here
          - <CORE-SNAP>
          - <CONTENT-SNAP-1>
          - <CONTENT-SNAP-2>
        override-prime: |
          set -eux
          for snap in "<CORE-SNAP>" "<CONTENT-SNAP-1>" "<CONTENT-SNAP-2>"; do  # List all content-snaps and base snaps you're using here
              cd "/snap/$snap/current" && find . -type f,l -exec rm -f "$SNAPCRAFT_PRIME/{}" \;
          done

2. List all the other parts of your snap in the ``after`` section, so that this part gets built last.

3. Replace ``<CORE-SNAP>`` in the ``build-snaps`` and ``override-prime`` sections with the ``base`` that you’re using.

4. List all your content snaps that you use in the ``build-snaps`` and ``override-prime`` sections.

..

   ℹ If your snap uses extensions, you can use the ``snapcraft expand-extensions`` command to see which content snaps the extension will add.

   ⓘ See :ref:`Desktop applications <desktop-applications>` for more information on how to snap a desktop application.
