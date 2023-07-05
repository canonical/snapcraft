.. 10818.md

.. _the-ffmpeg-sdk-stage-snaps:

The FFmpeg SDK stage snaps
==========================

These stage snaps let you compile your application against FFmpeg libraries.

NOTE
----

These snaps are still in development, their presentation, usage, and shipping content may change without notice.

Variants
--------

SDK stage snaps are named with ``ffmpeg-sdk-_variant_name_``, where ``_variant_name_`` is one of the following:

``lgpl``
~~~~~~~~

The linked product will become LGPL if it is statically linked against FFmpeg libraries.

``gplv2``
~~~~~~~~~

The linked product will become GPLv2 if it is linked against FFmpeg libraries.

``gplv3``
~~~~~~~~~

The linked product will become GPLv3 if it is linked against FFmpeg libraries.

How to use
----------

Add the following part definition to your snap’s Snapcraft YAML:

.. code:: yaml

   parts:
     ffmpeg:
       plugin: nil
       stage-snaps:
       - ffmpeg-sdk-_variant_

, add the ``ffmpeg`` part to your consuming parts’ ``after`` stanza:

.. code:: yaml

   parts:
     _consumer_part_name_:
       after:
       - ffmpeg

, then copy & paste the entire ``stage-packages`` stanza from the corresponding snapcraft source recipe to the ``ffmpeg`` part:

-  https://git.launchpad.net/ffmpeg-snaps/tree/snap/snapcraft.yaml?h=core18-lgpl-sdk
-  https://git.launchpad.net/ffmpeg-snaps/tree/snap/snapcraft.yaml?h=core18-gplv2-sdk
-  https://git.launchpad.net/ffmpeg-snaps/tree/snap/snapcraft.yaml?h=core18-gplv3-sdk

.. code:: yaml

   parts:
     ffmpeg:
       plugin: nil
       stage-snaps:
       - ffmpeg-sdk-_variant_
       stage-packages:
       - ...

, then merge the following fileset rules and prime rules so that only the required files will be placed into the snap:

::

       filesets:
         library-shared:
         - '**/lib/**/*.so*'

       prime:
       - $library-shared
