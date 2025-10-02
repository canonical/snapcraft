.. _reference-flutter-plugin:

Flutter plugin
==============

The Flutter plugin simplifies the building of applications that employ the `Flutter
<https://flutter.dev/>`_ UI toolkit.


Keys
----

This plugin provides the following unique keys for core22 and newer snaps.


flutter-channel
~~~~~~~~~~~~~~~

**Type**: ``stable``, ``master``, or ``beta``

**Default**: ``stable``

The flutter channel to use for the build.


flutter-target
~~~~~~~~~~~~~~

**Type**: string

**Default**: ``lib/main.dart``

The path to the app's entrypoint ``.dart`` file.


Dependencies
------------

For core22 and newer snaps, this plugin installs:

* ``clang``
* ``curl``
* ``git``
* ``cmake``
* ``ninja-build``
* ``unzip``.


How it works
------------

During the build step, the plugin performs the following actions:

* Clone `Flutter's GitHub repository <https://github.com/flutter/flutter>`_, using the
  channel declared by the ``flutter-channel`` key. If a channel isn't specified, the
  ``stable`` channel is cloned.
* Populate Flutter's cache of binary artifacts with ``flutter precache``.
* Download any dependencies listed in the Flutter project's ``pubspec.yaml`` with
  ``flutter pub get``.
* Build the app with ``flutter build``.
* Copy the resulting artifacts to the part's install directory.


Example
-------

See :ref:`how-to-craft-a-flutter-app` for an example of a snap built with the Flutter
plugin.
