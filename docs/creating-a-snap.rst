.. 6799.md

.. _creating-a-snap:

Creating a snap
===============

You can create snaps from apps you’ve already built and zipped, or from your preferred programming language or framework.

For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`.

The following is an outline of the typical snap build process, which you can step through to create your snap:

1. :ref:`Create a checklist <snapcraft-checklist>` Better understand your snap’s requirements.
2. :ref:`Create a snapcraft.yaml file <creating-snapcraft-yaml>` Describes your snap’s build dependencies and run-time requirements
3. :ref:`Add interfaces to your snap <adding-interfaces>` Share system resources with your snap, and from one snap to another
4. `Publish and share <https://snapcraft.io/docs/using-the-snap-store>`__ Put your snap on the `Snap Store <https://snapcraft.io/store>`__ to reach an audience of millions

Platform walkthroughs
---------------------

To get a quick hands-on synopsis of the snapcraft build process for your platform, choose a language or platform walk-through from the following:

+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| Common languages                | Platforms                                                     | Desktop toolkits                                                   |
+=================================+===============================================================+====================================================================+
| :ref:`Python <python-apps>`     | `Pre-built apps <https://snapcraft.io/docs/pre-built-apps>`__ | :ref:`Electron <electron-apps>`                                    |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Go <go-applications>`     | :ref:`MOOS <moos-applications>`                               | :ref:`GTK+ 4 <gtk4-applications>`                                  |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`.NET <net-apps>`          | :ref:`ROS <ros-deployment-with-snaps>`                        | :ref:`GTK+ 3 <gtk3-applications>`                                  |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Node.js <node-apps>`      | :ref:`ROS2 <ros-2-deployment-with-snaps>`                     | :ref:`GTK+ 2 <gtk2-applications>`                                  |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Ruby <ruby-applications>` |                                                               | :ref:`Java Swing <java-applications>`                              |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Rust <rust-applications>` |                                                               | :ref:`Qt 5 & KDE Frameworks <qt5-and-kde-frameworks-applications>` |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`C/C++ <c-c-applications>` |                                                               | :ref:`Flutter <flutter-applications>`                              |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Java <java-applications>` |                                                               |                                                                    |
+---------------------------------+---------------------------------------------------------------+--------------------------------------------------------------------+

.. toctree::
   :hidden:

   c-c-applications.rst
   electron-apps.rst
   flutter-applications.rst
   go-applications.rst
   gtk2-applications.rst
   gtk3-applications.rst
   gtk4-applications.rst
   java-applications.rst
   moos-applications.rst
   net-apps.rst
   node-apps.rst
   python-apps.rst
   qt5-and-kde-frameworks-applications.rst
   ros-2-deployment-with-snaps.rst
   ros-deployment-with-snaps.rst
   ruby-applications.rst
   rust-applications.rst
