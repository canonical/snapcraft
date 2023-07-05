.. 6233.md

.. _snap-confinement:

Snap confinement
================

One of the main motivations behind snaps is to *confine* applications, limiting their ability to access system resources. This is done to ensure that individual pieces of software do not impact the robustness of the user’s system or cause issues with other applications. As a result, when the user runs a snap, the software it provides is isolated from the system to some degree, with a default that limits access to a strict minimum of features.

Confinement levels
------------------

A snap’s confinement level is the degree of isolation it has from the user’s system. At a high level, knowing the confinement level enables users to understand the level of access an application has to their system. Application developers or packagers can adjust the confinement level to specify in broad terms how much access to system resources an application needs, either for normal use or during development.

There are three levels of snap confinement:

- **Strict** Used by the majority of snaps. Strictly confined snaps run in complete isolation, up to a minimal access level that’s deemed always safe. Consequently, strictly confined snaps can not access files, network, processes or any other system resource without requesting specific access via an interface (:ref:`see below <interfaces-and-confinement>`).
- **Classic** Allows access to the system’s resources in much the same way traditional packages do. To safeguard against abuse, publishing a classic snap requires :ref:`manual approval <process-for-reviewing-classic-confinement-snaps>`, and installation requires the ``--classic`` command line argument.
- **Devmode** A special mode for snap creators and developers. A *devmode* snap runs as a strictly confined snap with full access to system resources, and produces debug output to identify unspecified interfaces. Installation requires the ``--devmode`` command line argument. Devmode snaps cannot be released to the stable channel, do not appear in search results, and do not automatically refresh.

Strict confinement uses security features of the Linux kernel, including AppArmor, seccomp and namespaces, to prevent applications and services accessing the wider system.

Getting the confinement level
-----------------------------

You can discover the confinement level for any snap using the ``snap`` command:

.. code:: bash

   $ snap info --verbose vlc
   [...]
     confinement:       strict
     devmode:           false
   [...]

To see which installed snaps are using classic confinement, look for *classic* under the *Notes* column in the output of ``snap list``:

.. code:: bash

   $ snap list
   Name      Version   Rev   Tracking  Publisher       Notes
   vlc       3.0.6     770   stable    videolan✓       -
   code      0dd516dd  5     stable    vscode✓         classic
   wormhole  0.11.2    112   stable    snapcrafters    -

.. _interfaces-and-confinement:

Interfaces and confinement
--------------------------

Each snap’s interface is carefully selected by a snap’s creator to provide specific access to a resource, according to a snap’s requirements. Common interfaces provide :ref:`network access <the-network-interface>`, :ref:`desktop access <the-desktop-interfaces>` and :ref:`sound <the-pulseaudio-interface>` for example.

An interface needs to be connected to be active, and connections are made either automatically (at install time) or manually, depending on their function. The desktop interface is connected automatically, for instance, whereas the camera interface is not. See the *Auto-connect* column in :ref:`supported-interfaces` table for details on whether an interface automatically connects or not.

As with classic confinement, a snap’s publisher can request an *assertion* to automatically connect an otherwise non-auto-connecting interface. For example, the *guvcview* snap `requested <https://forum.snapcraft.io/t/auto-connect-request-for-the-guvcview-brlin-snap/6042>`__ the camera interface be automatically-connected when the snap is installed.

If a snap is upgraded and includes a new assertion, the user will still need to connect the interface manually. Similarly, if an installed classic snap is upgraded to use strict confinement, its interfaces won’t be automatically configured.

You can see which interfaces are connected and disconnected with the ``snap connections`` command (``vlc:camera`` is disconnected in the following example):

.. code:: bash

   $ snap connections vlc
   Interface         Plug                  Slot               Notes
   camera            vlc:camera            -                  -
   desktop           vlc:desktop           :desktop           -
   desktop-legacy    vlc:desktop-legacy    :desktop-legacy    -
   home              vlc:home              :home              -
   mount-observe     vlc:mount-observe     -                  -
   [...]

See :ref:`Interface management <interface-management>` for further interface details, including how to disconnect interfaces and make manual connections, and `Security policy and sandboxing <https://snapcraft.io/docs/security-policy-and-sandboxing>`__ for more information on how confinement is implemented.
