.. 13089.md

.. _the-audio-playback-interface:

The audio-playback interface
============================

The ``audio-playback`` interface allows a snap to play sounds and music, via the supporting audio service running on the system, such as PulseAudio. It’s used by many applications and utilities, and as such, is enabled by default.

Audio recording is enabled with the companion :ref:`audio-record <the-audio-record-interface>` interface, and unlike ``audio-playback``, is not enabled (auto-connected) by default.


Example
-------

The `VLC snap`_ is a good example of an application using the audio-playback interface:

.. code:: bash

   $ snap connections vlc
   Interface               Plug                        Slot                     Notes
   audio-playback          vlc:audio-playback          :audio-playback          -
   audio-record            vlc:audio-record            -                        -

If for some reason you want to disable audio playback for a snap, use the disconnect command:

.. code:: bash

   snap disconnect vlc:audio-playback

The connect command can be used to re-enable audio playback:

.. code:: bash

   snap connect vlc:audio-playback

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: yes


.. _the-audio-playback-interface-heading-code:

Code examples
~~~~~~~~~~~~~

The snapcraft.yaml for VLC_ includes audio-playback configuration: `https://github.com/videolan/vlc/blob/master/extras/package/snap/snapcraft.yaml <https://github.com/videolan/vlc/blob/75bca603749d8bfb7048a84ea811cbdb19447596/extras/package/snap/snapcraft.yaml#L36>`__

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/audio_playback.go

.. _`VLC snap`: https://snapcraft.io/vlc
.. _VLC: https://github.com/videolan/vlc
