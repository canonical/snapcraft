.. 13090.md

.. _the-audio-record-interface:

The audio-record interface
==========================

The ``audio-record`` interface allows an application to access your audio recording hardware, such as a microphone, via your system’s audio service, such as PulseAudio. It’s disabled by default.

This interface is a companion interface to the :ref:`audio-playback <the-audio-playback-interface>` interface, and is not intended to be used without it.


Example
-------

`OBS Studio`_ is a good example of an application that needs access to your microphone/audio recording hardware.

.. code:: bash

   $ sudo snap connect obs-studio:audio-record

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

The design of this interface is based on the principle that the slot implementation of the audio service, such as :ref:`PulseAudio <the-pulseaudio-interface>`, queries whether its audio-record slot is connected, leaving the audio service to mediate recording if it is.

On systems with snapd integration, PulseAudio’s mediation is limited and will only verify that the *snap* is connected to ``audio-record`` and not if the specific snap command plugs the interface.


Code examples
~~~~~~~~~~~~~

Mumble is a voice chat platform and a good example of an application using audio-record. Its snapcraft.yaml can be found here: `https://github.com/snapcrafters/mumble/blob/master/snap/snapcraft.yaml <https://github.com/snapcrafters/mumble/blob/b5f1644a72a14cacd17b862cd0265d21d8ce604a/snap/snapcraft.yaml#L21>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/audio_record.go

.. _`OBS Studio`: https://snapcraft.io/obs-studio
