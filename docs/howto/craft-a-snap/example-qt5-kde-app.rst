.. _example-qt5-kde-app:

Example Qt5 KDE app
===================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap for an app that uses Qt5 KDE. We'll work
through the aspects unique to Qt5 KDE apps by examining an existing recipe.


Example recipe for KCalc
------------------------

The following code comprises the recipe of a Qt5 KDE project, `KCalc
<https://github.com/KDE/kcalc>`_. This project is the calculator of KDE Gear.

.. collapse::

  .. code:: yaml

    name: kcalc
    version: '19.08.0'
    grade: stable
    adopt-info: kcalc

    confinement: strict
    base: core18

    apps:
        kcalc:
            command: kcalc
            extensions: [kde-neon]
            common-id: org.kde.kcalc.desktop
            plugs:
                - kde-frameworks-5-plug
                - home
                - opengl
                - network
                - network-bind
                - pulseaudio

    slots:
        session-dbus-interface:
            interface: dbus
            name: org.kde.kcalc.desktop
            bus: session

    parts:
        kcalc:
            plugin: cmake
            source: https://download.kde.org/stable/applications/19.08.0/src/kcalc-19.08.0.tar.xz
            parse-info:
                - usr/share/metainfo/org.kde.kcalc.appdata.xml
            build-snaps:
                - kde-frameworks-5-core18-sdk
                - kde-frameworks-5-core18
            build-packages:
              - libmpfr-dev
              - libgmp-dev
              - kdoctools-dev
            stage-packages:
              - libmpfr6
              - libgmp10
            configflags:
                - "-DKDE_INSTALL_USE_QT_SYS_PATHS=ON"
                - "-DCMAKE_INSTALL_PREFIX=/usr"
                - "-DCMAKE_BUILD_TYPE=Release"
                - "-DENABLE_TESTING=OFF"
                - "-DBUILD_TESTING=OFF"
                - "-DKDE_SKIP_TEST_SETTINGS=ON"


Add an app that uses KDE
------------------------

.. code:: yaml

  apps:
      kcalc:
          command: kcalc
          extensions: [kde-neon]
          common-id: org.kde.kcalc.desktop
          plugs:
              - kde-frameworks-5-plug
              - home
              - opengl
              - network
              - network-bind
              - pulseaudio

Apps that use KDE runtime libraries require the `kde-neon
<https://snapcraft.io/docs/kde-neon-extension>`_. The extension configures the
runtime environment of the app so that all desktop functionality is correctly
initialised. As desktop environment apps, they also need special configuration
for AppStream and ``.desktop`` file compatibility.

To add a GTK4 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. For ``extensions``, add ``kde-neon``.
#. Set ``common-id`` to the app's unique AppStream ID. Doing so links the app
   to the ``.desktop`` launcher specified in the AppStream file.


Add a part written for Qt5 KDE
------------------------------

.. code:: yaml

    parts:
      kcalc:
          plugin: cmake
          source: https://download.kde.org/stable/applications/19.08.0/src/kcalc-19.08.0.tar.xz
          parse-info:
              - usr/share/metainfo/org.kde.kcalc.appdata.xml
          build-snaps:
              - kde-frameworks-5-core18-sdk
              - kde-frameworks-5-core18
          build-packages:
            - libmpfr-dev
            - libgmp-dev
            - kdoctools-dev
          stage-packages:
            - libmpfr6
            - libgmp10
          configflags:
              - "-DKDE_INSTALL_USE_QT_SYS_PATHS=ON"
              - "-DCMAKE_INSTALL_PREFIX=/usr"
              - "-DCMAKE_BUILD_TYPE=Release"
              - "-DENABLE_TESTING=OFF"
              - "-DBUILD_TESTING=OFF"
              - "-DKDE_SKIP_TEST_SETTINGS=ON"

Qt5 KDE parts don't require a special plugin. Instead, they need extra snap dependencies.

To add a Qt5 KDE part:

#. Declare the general part keys, such as ``plugin``, ``source``,
   ``build-packages``, and so on.
#. So that the app has access to its AppStream metadata, for ``parse-info`` add
   a path to the AppStream ``.xml`` file on the host system. Since we set
   ``adopt-info: kcalc`` at the start of the recipe, the AppStream file of the
   ``kcalc`` part will be used to fill in the ``summary``, ``description`` and
   ``icon`` of this snap and copy the AppStream file. See `Using AppStream
   metadata
   <https://snapcraft.io/docs/using-external-metadata#heading--appstream>`_ for
   technical details about how this works.
#. For ``build-snaps``, list the following dependencies:

   - ``kde-frameworks-5-core18-sdk``
   - ``kde-frameworks-5-core18``
