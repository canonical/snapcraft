.. 13123.md

.. _adding-interfaces:

Adding interfaces
=================

After :ref:`Defining a command <defining-a-command>`, *interfaces* are the means by which an installed snap gets access to system resources. Interfaces that are required for normal operation are specified at snap build-time within the :ref:`app and service metadata <snapcraft-app-and-service-metadata>` of a snap’s :ref:`snapcraft.yaml <creating-snapcraft-yaml>`.

Many interfaces are automatically connected when a snap is installed, but this ability is dependent on the permissiveness of each particular interface. See :ref:`Auto-connections <interface-management-auto-connections>` for more details.

Other interfaces require the user to make a manual connection, such as :ref:`camera <the-camera-interface>` and :ref:`removable-media <the-removable-media-interface>`. Manual connections enable the user to have a complete control over what kind of access they allow.

Once published in the `Snap Store <https://snapcraft.io/store>`__, automatic connections may be requested for manual interfaces on a case-by-case basis. For example, it may be reasonable for a photo-booth application to expect an automatic connection to the ``camera`` interface. Those requests are submitted and processed in the open on the Snapcraft forum. For more details on this process, see :ref:`Permission requests <permission-requests>` .

Plugs and slots
---------------

Interfaces are implemented as ``plugs`` and ``slots``. A plug in one snap may connect to a slot in another and this provides access to the resources required. For example the ``ohmygiraffe`` snap specifies the ``opengl`` plug which connects to the ``opengl`` slot provided elsewhere - in this case by the ``core`` snap.

Users may view the interfaces used as a list of ``plugs`` and ``slots`` via the ``snap connections`` command:

.. code:: bash

   $ snap connections ohmygiraffe
   Interface     Plug                      Slot           Notes
   network       ohmygiraffe:network       :network       -
   network-bind  ohmygiraffe:network-bind  :network-bind  -
   opengl        ohmygiraffe:opengl        :opengl        -
   pulseaudio    ohmygiraffe:pulseaudio    :pulseaudio    -
   x11           ohmygiraffe:x11           :x11           -

Developers specify a list of ``plugs`` for each application inside a snap being exposed to the host OS. Each application may have a different set of ``plugs`` specified. Developers should endeavour to list only the ``plugs`` required for normal operation of the application.

In a simplified example, a snap which contains both a server, which connects to a webcam to stream online, and a graphical application, to view the camera, may have interfaces listed as follows:

.. code:: yaml

   apps:
     streamer:
       command: bin/streamer-cli
       plugs:
         - network-bind
         - camera
     frontend:
       command: bin/frontend-gui
       plugs:
         - network
         - camera
         - x11

..

   ℹ The state of a connection can be checked within a snap using the *snapctl* utility. See `Using the snapctl tool <https://snapcraft.io/docs/using-the-snapctl-tool>`__ for further details.

Common interfaces
-----------------

When building a snap, its interfaces will be as unique as its application requirements. You can use the :ref:`snappy-debug <debugging-building-snaps-identifying-missing-interfaces>` tool to figure out which interfaces a snap needs.

The `FFmpeg <https://snapcraft.io/ffmpeg>`__ multimedia framework, for example, needs interfaces for audio, USB cameras, network access and the desktop, `among many others <https://github.com/snapcrafters/ffmpeg/blob/master/snap/snapcraft.yaml>`__. The game `Spelunky <https://snapcraft.io/spelunky>`__ needs to `access <https://github.com/snapcrafters/spelunky/blob/master/snap/snapcraft.yaml>`__ OpenGL, the desktop environment and any connected joystick.

The process of adding interfaces requires the snap developer to have a good understanding of the applications it contains, but there are certain categories of snap that require the same, or very similar, sets of interfaces.

Being familiar with these can help to speed up snap development:

-  :ref:`Games interfaces <games-interfaces>`
-  :ref:`Desktop interfaces <the-desktop-interfaces>`
-  :ref:`Network interface <network-interface>`

See :ref:`Supported interfaces <supported-interfaces>` for the full list of interfaces available for snaps to use.
