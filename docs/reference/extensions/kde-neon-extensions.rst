.. _reference-kde-neon-extensions:

KDE neon extensions
===================

The KDE neon extensions help authors create snaps that use `Qt <https://doc.qt.io>`_,
with or without the `KDE Frameworks <https://develop.kde.org/products/frameworks>`_
libraries. This page covers the architecture of the extension and its effect on project
files during build.

This a family of extensions for two versions of KDE neon paired with Qt:

- KDE neon 6, internally named ``kde-neon-6``
- KDE neon 5, internally named ``kde-neon``


Compatibility
-------------

The KDE neon 6 extension is supported with the core24 and core22 bases.

The KDE neon 5 extension is supported with the core22 base. An older version of
the extension is also available for core20.

.. important::

    As of February 2024, KDE no longer supports KDE neon 5.

.. _kde-neon-extensions-libraries:

.. list-table::

    * - Base
      - Versions
      - Platform snap
      - Build snap
    * - core24
      - Qt 6 and KDE Frameworks 6
      - `kf6-core24 <https://snapcraft.io/kf6-core24>`_
      - `kf6-core24-sdk <https://snapcraft.io/kf6-core24-sdk>`_
    * - core22
      - Qt 6 and KDE Frameworks 6
      - `kf6-core22 <https://snapcraft.io/kf6-core22>`_
      - `kf6-core22-sdk <https://snapcraft.io/kf6-core22-sdk>`_
    * - core22
      - Qt 5.15.11 and KDE Frameworks 5.113
      - `kf5-5-113-qt-5-15-11-core22
        <https://snapcraft.io/kf5-5-113-qt-5-15-11-core22>`_
      - `kf5-5-113-qt-5-15-11-core22-sdk
        <https://snapcraft.io/kf5-5-113-qt-5-15-11-core22-sdk>`_

The extensions are designed for C++ based Qt/KDE Frameworks apps. They don't provide the
bindings needed for PySide2 (Qt for Python) or PyQt apps. The extensions also don't
provide all of the optional Qt libraries. For example, they don't include Qt3D,
QtCharts, QtDataVisualization or QtGamepad.


Included interfaces
-------------------

The KDE neon 6 extension connects the snap to the following runtime content snaps:

- The :ref:`snaps <kde-neon-extensions-libraries>` for Qt and KDE Frameworks
  run-time libraries.
- The `gtk-common-themes <https://snapcraft.io/gtk-common-themes>`_ snap for common
  icon, cursor and sound themes.

The extension automatically includes the required plugs for these runtime libraries.

.. dropdown:: Included interfaces from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          desktop:
            mount-host-font-cache: false
          gtk-2-themes:
            interface: content
            target: $SNAP/data-dir/themes
            default-provider: gtk-common-themes
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
          kf6-core24:
            content: kf6-core24-all
            interface: content
            target: $SNAP/kf6
            default-provider: kf6-core24
          gpu-2404:
            interface: content
            target: $SNAP/gpu-2404
            default-provider: mesa-2404

In addition, the extension adds the following plugs to each app that uses it:

.. dropdown:: Included plugs for apps from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          - desktop
          - desktop-legacy
          - opengl
          - wayland
          - x11
          - audio-playback
          - unity7
          - network
          - network-bind


Included packages
-----------------

The KDE neon 6 extension depends on two separate snaps -- a build snap and a combination
platform-content snap.

The build snap ensures that the relevant Qt and KDE Frameworks development libraries and
supporting files are available during the build process. These libraries are sourced
from the `KDE neon project <https://neon.kde.org>`_, which provides more recent versions
of Qt and the KDE Frameworks.

The platform snap makes the corresponding run-time libraries available to the snap when
it's launched by the user. If the platform snap isn't already present on a user's
machine, then it will be installed automatically and simultaneously with the KDE neon
extension-based snap.

By relying on a standalone platform snap, authors can avoid bundling the Qt and KDE
Frameworks libraries in their snap, keeping the file size of the snap to a minimum. The
same installation of the platform snap can be used by multiple snaps that rely on the
KDE neon extensions.


Included build environment variables
------------------------------------

The KDE neon 6 extension sets environment variables so the project's parts can build
correctly. The project's main part receives the following variables.

.. dropdown:: Included build environment variables from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        build-environment:
          - PATH: /snap/kde-qt6-core24-sdk/current/usr/bin:/snap/kf6-core24-sdk/current/usr/bin${PATH::$PATH}
          - XDG_DATA_DIRS: $CRAFT_STAGE/usr/share:/snap/kde-qt6-core24-sdk/current/usr/share:/snap/kf6-core24-sdk/current/usr/share:/usr/share${XDG_DATA_DIRS::$XDG_DATA_DIRS}
          - XDG_CONFIG_HOME: $CRAFT_STAGE/etc/xdg:/snap/kde-qt6-core24-sdk/current/etc/xdg:/snap/kf6-core24-sdk/current/etc/xdg:/etc/xdg${XDG_CONFIG_HOME::$XDG_CONFIG_HOME}
          - LD_LIBRARY_PATH: /snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:/snap/kde-qt6-core24-sdk/current/usr/lib:/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas:/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack:/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libproxy:/snap/kf6-core24-sdk/current/usr/lib:$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:$CRAFT_STAGE/usr/lib:$CRAFT_STAGE/lib/${LD_LIBRARY_PATH::$LD_LIBRARY_PATH}
          - CMAKE_PREFIX_PATH: $CRAFT_STAGE;/snap/kde-qt6-core24-sdk/current;/snap/kf6-core24-sdk/current;/usr${CMAKE_PREFIX_PATH:;$CMAKE_PREFIX_PATH}
          - CMAKE_FIND_ROOT_PATH: $CRAFT_STAGE;/snap/kde-qt6-core24-sdk/current;/snap/kf6-core24-sdk/current;/usr${CMAKE_FIND_ROOT_PATH:;$CMAKE_FIND_ROOT_PATH}


Included runtime environment variables
--------------------------------------

The KDE neon 6 extension also set various runtime environment variables for apps.

The main runtime variables apply to the whole snap.

.. dropdown:: Included snap-wide runtime variables from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        environment:
          SNAP_DESKTOP_RUNTIME: $SNAP/kf6
          GTK_USE_PORTAL: "1"
          QT_VERSION: "6"

.. _kde-neon-extensions-configure-hook:

The rest of the runtime variables are set at launch by a ``command-chain`` shell script
included by a :ref:`configure hook <how-to-add-a-snap-configuration-configure-hook>`:

.. dropdown:: Included configure hook from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        hooks:
          configure:
            command-chain:
              - snap/command-chain/hooks-configure-desktop

The KDE neon 6 extension adds this shell script to the snap through a special part.

.. dropdown:: Included SDK build part from KDE neon 6

    .. code-block:: yaml
        :caption: snapcraft.yaml

        parts:
          ...
          kde-neon-6/sdk:
            source: /snap/snapcraft/current/share/snapcraft/extensions/desktop/command-chain-kde

The part assembles the shell script from shared scripts in Snapcraft's source:

- `Common desktop scripts
  <https://github.com/canonical/snapcraft/blob/main/extensions/desktop/common>`_
- `KDE neon 6 scripts
  <https://github.com/canonical/snapcraft/blob/main/extensions/desktop/kde-neon-6>`_


Included layouts
----------------

The KDE neon 6 extension adds the following :ref:`layouts <reference-layouts>`.

.. dropdown:: Included layouts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        layout:
          /usr/share/X11:
            symlink: $SNAP/kf6/usr/share/X11
          /usr/share/qt6:
            symlink: $SNAP/kf6/usr/share/qt6
          /usr/share/libdrm:
            bind: $SNAP/gpu-2404/libdrm
          /usr/share/drirc.d:
            symlink: $SNAP/gpu-2404/drirc.d


Included hooks
--------------

The KDE neon 6 extension adds a :ref:`hook <kde-neon-extensions-configure-hook>` that
sets runtime environment variables.


Example expanded project files
------------------------------

Here are examples of the result of a project file that uses the KDE neon
extensions. They demonstrate the added plugs, packages, variables, and layouts
that the extensions add to project files immediately prior to build.

These examples contain the difference between the original files and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

.. tab-set::

    .. tab-item:: KDE neon 6

        The original is a `project file
        <https://invent.kde.org/alexlowe/keysmith/-/blob/aml/snapcraft/snapcraft.yaml>`_
        for a snapped version of `Keysmith <https://apps.kde.org/keysmith/>`_.

        .. dropdown:: Expanded project file for Keysmith

            .. literalinclude:: code/kde-neon-6-extension-keysmith-expanded.diff
                :language: diff
                :lines: 3-
                :emphasize-lines: 45-51, 65-122, 130-138, 140-142, 144-171, 179-182

    .. tab-item:: KDE neon 5

        The original project file comes from the `KCalc snap
        <https://snapcraft.io/kcalc>`_.

        We provide a review of the unmodified file in :ref:`how-to-craft-a-qt5-kde-app`.

        .. dropdown:: Expanded project file for KCalc

            .. literalinclude:: code/kde-neon-extension-kcalc-expanded.diff
                  :language: diff
                  :lines: 3-
                  :emphasize-lines: 15-19, 27-28, 58-103
