.. 11702.md

.. _desktop-app-support-qt-for-snaps-without-bases:

Desktop App support - Qt (for snaps without bases)
==================================================

   Snaps without specifying base snap is now a deprecated practice, publishers are advised to migrate to the syntax with the ``base`` property set as soon as possible

Graphical applications which use the Qt require additional libraries and environment configuration to function correctly inside a snap.

The details have been collected into a remote part, which can be included in any snap at build time.

Qt4 based applications should use the ``desktop-qt4`` remote part. Qt5 applications should reference the ``desktop-qt5`` part.

In this snippet the remote part is referenced in an ``after`` section.

::

   parts:
       audiocoder:
           after: [desktop-qt5]

In addition the launcher script which sets up the environment should prefix the binary name in the ``apps`` section. Typically it’s also necessary to specify the full path to the target binary after the launcher.

::

   apps:
      command: desktop-launch $SNAP/usr/bin/audiocoder-qt
          plugs: [network, desktop, desktop-legacy]

These pre-defined parts do not automatically pull in all necessary Qt libraries, but the minimum required by most applications.

The developer is expected to list any further libraries as ``stage-packages`` or additional ``parts``.
