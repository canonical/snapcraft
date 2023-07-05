.. 6545.md

.. _using-chromium-ffmpeg-in-third-party-browser-snaps:

Using chromium-ffmpeg in third-party browser snaps
==================================================

The `chromium snap <https://snapcraft.io/chromium>`__ ships FFMpeg codecs that allow decoding videos in many popular formats, including VP8, VP9, H.264 and Theora.

Other third-party browser snaps or snaps that embed a webview based on chromium may not be allowed to redestribute those codecs for legal reasons. For those, there is a `chromium-ffmpeg <https://snapcraft.io/chromium-ffmpeg>`__ content snap that exposes slots which snaps can connect to to see a copy of the desired version of ``libffmpeg.so``.

The provider declares several slots:

::

   $ snap connections chromium-ffmpeg
   Interface  Plug  Slot                                    Notes
   content    -     chromium-ffmpeg:chromium-ffmpeg-108372  -
   content    -     chromium-ffmpeg:chromium-ffmpeg-111306  -

Each slot has an FFMpeg version number in its name, which corresponds to a given `chromium release <https://chromium.googlesource.com/chromium/third_party/ffmpeg.git/+log/HEAD/chromium/config/Chrome/linux/x64/libavutil/ffversion.h>`__. - FFMpeg 108372: chromium 105 - FFMpeg 111306: chromium 114

A consumer snap needs to declare a corresponding plug in its ``snapcraft.yaml`` file:

::

   plugs:
     chromium-ffmpeg-111306:
       interface: content
       target: $SNAP
       default-provider: chromium-ffmpeg

Once the connection is made, the consumer will see ``libffmpeg.so`` mounted at ``$SNAP/chromium-ffmpeg/libffmpeg.so``. By appending ``$SNAP/chromium-ffmpeg`` to ``LD_LIBRARY_PATH`` (or whatever other shared library loading mechanism the consumer uses to look up and load ``libffmpeg.so``), the codecs will be made available to the application.

To make the connection:

::

   snap connect <consumer-snap-name>:chromium-ffmpeg-111306 chromium-ffmpeg:chromium-ffmpeg-111306

Note that consumer snaps that are in the store can request an assertion to be set up so that the connection will be made automatically when the snap is installed. To enable this, the maintainer of the snap should create a forum post in `the store-requests category <https://forum.snapcraft.io/c/store-requests>`__ to make the case for the auto-connection.
