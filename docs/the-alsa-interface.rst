.. 7766.md

.. _the-alsa-interface:

The alsa interface
==================

``alsa`` allows access to raw ALSA audio playback and recording devices. This is equivalent to direct driver access to your audio hardware and may block other applications from recording or playing sound.

To provide better audio sharing and input and output configuration, it’s recommended that most snaps access audio functionality through the system audio layer, such as PulseAudio, via the :ref:`audio-playback <the-audio-playback-interface>` and :ref:`audio-record <the-audio-record-interface>` interfaces.

However, raw access to ALSA devices using this interface can provide a slight performance advantage with input and output latency and avoid resampling which can reduce audio quality.


Example
-------

The musical notation and composition application, MuseScore_, is a good example of a snap that uses the ALSA interface.

To connect a snap to the ALSA interface, run the following command:

.. code:: bash

   $ sudo snap connect <snap name>:alsa

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

The ``alsa`` interface is not auto-connected, in part, because not all hardware will multiplex clients and therefore may block audio.

The *libasound2* library needs to be included in a snap’s ``stage-packages``, of the part which uses ALSA, either directly or through some other package which brings it in (or manually compiled).


Code examples
~~~~~~~~~~~~~

The :file:`snapcraft.yaml` for MuseScore includes an ALSA interface definition: `https://github.com/pachulo/musescore-snap/blob/master/snap/snapcraft.yaml <https://github.com/pachulo/musescore-snap/blob/9d328cb48679542180b257e32131bbf23ea8cba0/snap/snapcraft.yaml#L32>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/alsa.go

.. _MuseScore: https://snapcraft.io/musescore
