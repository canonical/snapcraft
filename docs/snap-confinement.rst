.. 6233.md

.. _snap-confinement:

Snap confinement
================

Snap confinement determines the amount of access an application has to system
resources, such as files, the network, peripherals and services. There are
several levels of confinement.

Confinement ensures that individual pieces of software do not impact the
robustness of the user's system or cause issues with other applications.
As a result, when the user runs a snap, the software it provides is isolated
from the system to some degree, with a default that limits access to a strict
minimum of features.

Confinement levels
------------------

A snap's confinement level controls the degree of isolation it has from the
user's system. Application developers or packagers can adjust the confinement
level to specify in broad terms how much access to system resources an
application needs, either for normal use or during development.

There are two levels of snap confinement for published snaps: strict and
classic. These are designed to meet the needs of different types of
applications. An additional mode, devmode, is useful during the development
process.


Strict
~~~~~~

This level is used by the majority of snaps. Strictly confined snaps run in
complete isolation, up to a minimal access level that is considered safe.
As a result, strictly confined snaps cannot access files, network, processes or
any other system resources without requesting specific access via an interface
(:ref:`see below <interfaces-and-confinement>`).

Strict confinement uses security features of the Linux kernel, including
AppArmor, seccomp and namespaces, to prevent applications and services
accessing the wider system.

Classic
~~~~~~~

Allows access to the system's resources in a similar way to traditional
packages. To safeguard against abuse, publishing a classic snap requires
:ref:`manual approval <process-for-reviewing-classic-confinement-snaps>`, and
installation requires the ``--classic`` command line argument to be passed to
the :command:`snap` command.

See :ref:`classic-confinement` for the features and limitations of this mode.

Developer mode (devmode)
~~~~~~~~~~~~~~~~~~~~~~~~

A special mode for snap creators and developers, a devmode snap runs as a
strictly confined snap with full access to system resources, and produces debug
output to identify unspecified interfaces. Installation requires the
``--devmode`` command line argument to be passed to the :command:`snap`
command. Devmode snaps cannot be released to the stable channel, do not appear
in search results, and do not automatically refresh.

.. Apparently, devmode snaps can only be published to the edge channel...

Getting the confinement level
-----------------------------

Use the :command:`snap info` command to discover the confinement level for a
snap:

.. code:: bash

   $ snap info --verbose vlc
   [...]
     confinement:       strict
     devmode:           false
   [...]

To see which installed snaps are using classic confinement, look for
*classic* under the *Notes* column in the output of the :command:`snap list`
command:

.. code:: bash

   $ snap list
   Name      Version   Rev   Tracking  Publisher       Notes
   vlc       3.0.6     770   stable    videolan✓       -
   code      0dd516dd  5     stable    vscode✓         classic
   wormhole  0.11.2    112   stable    snapcrafters    -

.. _interfaces-and-confinement:

Interfaces and confinement
--------------------------

Snaps with strict confinement must use :ref:`interfaces <interfaces>` to
access resources on the user's system, including those provided by other snaps.
