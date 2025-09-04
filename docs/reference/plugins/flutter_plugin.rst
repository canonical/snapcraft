.. _reference-flutter-plugin:

Flutter plugin
==============

The Flutter plugin builds `Flutter <https://flutter.dev/>`_-based parts


Keywords
--------

This plugin provides the following unique keys for core22 and core24 snaps.


flutter-channel
~~~~~~~~~~~~~~~
**Type**: ``stable``, ``master``, or ``beta``

**Default**: ``stable``

The default flutter channel to use for the build.


flutter-target
~~~~~~~~~~~~~~
**Type**: string

**Default**: ``lib/main.dart``

The flutter target to build.


Dependencies
------------

For core22 and core24 snaps, this plugin installs ``clang``, ``curl``, ``git``,
``cmake``, ``ninja-build``, and ``unzip``.


How it works
------------

During the build step, the plugin performs the following actions:

#. Clone `Flutter's GitHub repository <https://github.com/flutter/flutter>`_, using the
   channel declared by the ``flutter-channel`` key. If a channel isn't specified, the
   ``stable`` channel is cloned.
#. Remove ``flutter-distro/engine/src/.gn`` from the part's build directory so that
   subsequent commands run successfully.
#. Populate Flutter's cache of binary artifacts with ``flutter precache``.
#. Download any dependencies listed in the Flutter project's ``pubspec.yaml`` with
   ``flutter pub get``.
#. Build the target declared by ``flutter-target`` with ``flutter build``. If no target
   is specified, ``lib/main.dart`` is built.
#. Copy the resulting artifacts to the part's install directory.


Example
-------

See :ref:`how-to-craft-a-flutter-app` for an example of a snap built with the Flutter
plugin.
