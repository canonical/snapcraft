.. 6273.md

.. _adding-opengl-gpu-support-to-a-snap:

Adding OpenGL/GPU support to a snap
===================================

Some applications and games use OpenGL, Vulkan or CUDA libraries to access GPU(s). Typically, snapped applications require three things to make this possible.

1. Including additional user space GPU libraries.
2. Initialising those libraries before your application starts.
3. Permitting access using the ``opengl`` interface.

This document explains how to do this.

Include user space GPU libraries
--------------------------------

You need to include the appropriate user space “drivers” in the snap to expose OpenGL/GPU capabilities. All that is typically required is the ``libglu1-mesa`` package. Add this to the ``stage-packages`` of the ``part`` of your application.

.. code:: yaml

   parts:
     mygame:
       stage-packages:
         - libglu1-mesa

Configure environment
---------------------

Graphical applications and GPU libraries require environment configuration to function correctly. The `desktop helpers project <https://github.com/ubuntu/snapcraft-desktop-helpers>`__ provides scripts that do additional setup to ensure toolkits, audio stacks and graphics drivers are correctly configured when your application starts. If your application uses :ref:`GTK <desktop-app-support-gtk>`, :ref:`Qt <desktop-app-support-qt>`, or :ref:`another desktop toolkit <desktop-applications>`, then follow the instructions for that toolkit. The ``desktop-launch`` script for that toolkit will also take care of GPU library configuration.

If your application, like most games, does not use a common desktop toolkit, then you can use the ``desktop-glib-only`` helper to do the initialisation of desktop features such as GPU, sound and fonts. The first step is to include a dependency on the ``desktop-glib-only`` part in the part of your application.

.. code:: yaml

   parts:
     mygame:
       after:
         - desktop-glib-only

The next step is to copy the ``desktop-glib-only`` part from the :file:`snapcraft.yaml` file in the `desktop helpers`_ repository.

.. code:: yaml

     # This part installs the glib dependencies and a `desktop-launch` script to initialise
     # desktop-specific features such as OpenGL, fonts, themes and the XDG environment.
     #
     # It is copied from the snapcraft desktop helpers project and the `source` key is
     # changed to point to https://github.com/ubuntu/snapcraft-desktop-helpers.git
     # Please periodically check the desktop helpers repo for updates and copy the changes.
     #    https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml
     #
     desktop-glib-only:
       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
       source-subdir: glib-only
       ...
       <copy the rest of this part from the desktop helpers snapcraft.yaml>
       ...

If none if the prebuilt helpers are suitable for your application, you can create a helper script yourself. See the ``sommelier`` script in the `Track Mania Nation Forever snap <https://github.com/snapcrafters/tmnationsforever>`__ as an example.

Permit access to GPU hardware
-----------------------------

The :ref:`snap sandbox <snap-confinement>` doesn’t allow access to GPU hardware by default. You can enable this access by adding the :ref:`opengl interface <the-opengl-interface>` to the ``plugs`` section of your application in ``apps``.

.. code:: yaml

   apps:
     mygame:
       plugs:
         - opengl

Learn more about :ref:`interfaces and the available plugs <interface-management>`. Including other :ref:`desktop interfaces <the-desktop-interfaces>` might be required, for example to connect to the X server.

More information
----------------

-  :ref:`Overview of snapcraft support for graphical applications and toolkits <desktop-applications>`
-  See the `snapcraft.yaml for Xonotic <https://github.com/snapcrafters/xonotic/blob/master/snap/snapcraft.yaml>`__ for a more complete example.

.. _`desktop helpers`: https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml
