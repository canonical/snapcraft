.. 6799.md

.. _creating-a-snap:

Creating a snap
===============

You can create snaps from apps you've already built and zipped, or from your
preferred programming language or framework.

For a brief overview of the snap creation process, including how to install
Snapcraft and how it is used, see :ref:`snapcraft-overview`.

The following is an outline of the typical snap build process, which you can
step through to create your snap:

#. :ref:`Create a checklist <snapcraft-checklist>` to better understand your snap's requirements.
#. :ref:`Create a snapcraft.yaml file <creating-snapcraft-yaml>` to describe
   your snap's build and run-time requirements.
#. :ref:`Add interfaces to your snap <adding-interfaces>` to share resources
   between snaps and the user's system.
#. `Publish and share <https://snapcraft.io/docs/using-the-snap-store>`_ your
   snap on the `Snap Store <https://snapcraft.io/store>`_.

This should give you an idea of the process behind the creation of a snap.

Platform walkthroughs
---------------------

Applications are often built using programming languages and technologies that
are supplied with their own build tools. Snapcraft uses :term:`plugins` to
integrate with these tools in order to create snaps.

Choose a walkthrough from the following list to see how to create a snap for
a particular language, platform or technology:

.. toctree::
   :maxdepth: 1

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
   pre-built-apps.rst
   python-apps.rst
   qt5-and-kde-frameworks-applications.rst
   ros-2-deployment-with-snaps.rst
   ros-deployment-with-snaps.rst
   ruby-applications.rst
   rust-applications.rst
