.. 13125.md

.. _games-interfaces:

Games interfaces
================

Games lend themselves to being snapped quite well. Game developers often have focus on Windows and MacOS, so packaging their creations for multiple Linux distributions can be daunting and time consuming. Games typically require a similar set of interfaces to enable their main functions. Most games require being able to graphically draw on the screen, play audio via the sound card, and perhaps access the network for multi-player features.

Snaps provide a set of interfaces which likely cover most of the requirements for modern games.

::

   apps:
     3dgame:
       command: bin/gamebinary
       plugs:
         - x11
         - opengl
         - network
         - network-bind
         - audio-playback
         - joystick

In addition a small set of ``stage-packages`` are typically required as modern OpenGL games usually depend on this bare minimum for access to the above resources via interfaces.

::

   parts:
     3dgame:
       stage-packages
         - libpulse0
         - libglu1-mesa
         - libgl1
     ...

Depending on how the game is built, and what libraries it already bundles, it may be necessary to add other ``stage-packages`` which cannot be guaranteed to be installed on the host. Modern versions of snapcraft will highlight the missing libraries as a pastable list which can be added to the existing :file:`snapcraft.yaml` file.
