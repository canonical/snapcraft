.. 20521.md

.. _supported-extensions:

Supported extensions
====================

Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. For more information on how to use extensions, see :ref:`Snapcraft extensions <snapcraft-extensions>`.

The following extensions are available in the latest version of :ref:`Snapcraft <snapcraft-overview>`.

.. list-table::
   :header-rows: 1

   * - Extension name
     - Description
     - :ref:`Supported base <base-snaps>`
   * - :ref:`flutter-stable <the-flutter-extension>`
     - Create snaps that track the *stable* channel of the `Flutter UI toolkit`_
     - core18
   * - :ref:`flutter-beta <the-flutter-extension>`
     - Create snaps that track the *beta* channel of the `Flutter UI toolkit`_
     - core18
   * - :ref:`flutter-dev <the-flutter-extension>`
     - Create snaps that track the *dev* channel of the `Flutter UI toolkit`_
     - core18
   * - :ref:`flutter-master <the-flutter-extension>`
     - Create snaps that track the *master* channel of the `Flutter UI toolkit`_
     - core18
   * - :ref:`gnome <the-gnome-extension>`
     - Create snaps that integrate with GNOME and GTK. Also useful for general desktop applications.
     - core22
   * - :ref:`gnome-3-28 <the-gnome-3-28-extension>`
     - This older version of the ``gnome`` extension supports GTK 3.28 and core18
     - core18
   * - :ref:`gnome-3-34 <the-gnome-3-34-extension>`
     - This older version of the ``gnome`` extension supports GTK 3.34 and core18
     - core18
   * - :ref:`gnome-3-38 <the-gnome-3-38-extension>`
     - This older version of the ``gnome`` extension supports GTK 3.38 and core20
     - core20
   * - :ref:`kde-neon <the-kde-neon-extension>`
     - Create snaps of desktop applications that use Qt5 and/or `KDE Frameworks <https://kde.org/products/frameworks/>`__
     - core18 core20 core22
   * - :ref:`ros1-noetic <the-ros-1-noetic-extension>`
     - This extension helps you snap ROS 1 applications for the `Noetic Ninjemys <https://wiki.ros.org/noetic>`__ distribution *(experimental)*
     - core20
   * - :ref:`ros2-foxy <the-ros2-foxy-extension>`
     - This extension helps you snap ROS 2 applications for the `Foxy Fitzroy <https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/>`__ distribution *(experimental)*
     - core20
   * - :ref:`ros2-humble <the-ros-2-humble-extension>`
     - This extension helps you snap ROS 2 applications for the `Humble Hawksbill <https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html>`__ distribution *(experimental)*
     - core22

The *snapcraft extensions* command lists which extensions are supported by the installed version of snapcraft.

.. toctree::
   :hidden:

   the-flutter-extension
   the-gnome-3-28-extension
   the-gnome-3-34-extension
   the-gnome-3-38-extension
   the-gnome-extension
   the-kde-neon-extension
   the-ros-1-noetic-extension
   the-ros2-foxy-extension
   the-ros-2-humble-extension
