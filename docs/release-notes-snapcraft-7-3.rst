.. 34629.md

.. _release-notes-snapcraft-7-3:

Release notes: Snapcraft 7.3
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 7.3 <https://github.com/snapcore/snapcraft/releases/tag/7.3>`__, a major update to the tool used to build snap packages.

For general details, including installation instructions, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__, or take a look at `Snapcraft release notes <https://snapcraft.io/docs/snapcraft-release-notes>`__ for other *Snapcraft* releases.

Plugins
-------

-  New :ref:`Flutter plugin <the-flutter-plugin>` with core22 support `#3952 <https://github.com/snapcore/snapcraft/pull/3952>`__

Extensions
----------

-  Removed experimental flag `#3988 <https://github.com/snapcore/snapcraft/pull/3988>`__
-  New ``core22`` extension for :ref:`kde-neon <the-kde-neon-extension>` , with new content :ref:`kde-neon <the-kde-neon-extension>` and KDE Frameworks (kf5 5.98 and qt 5.15.6) content snaps `#3991 <https://github.com/snapcore/snapcraft/pull/3991>`__
-  Minimum necessary build packages are now being used for the :ref:`ROS2 Foxy extension <the-ros2-foxy-extension>` (core20) `#4019 <https://github.com/snapcore/snapcraft/pull/4019>`__

Build Providers
---------------

-  set environment according to directory inside instances `#3951 <https://github.com/snapcore/snapcraft/pull/3951>`__
-  pass ``SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT`` to instance by `#3997 <https://github.com/snapcore/snapcraft/pull/3997>`__

Parts
-----

-  Added ``core22`` support to the :ref:`SCons <the-scons-plugin>` plugin
-  Added ``core22`` support to the :ref:`Ant plugin <the-ant-plugin>`
-  Added ``core22`` support to the :ref:`Maven plugin <the-maven-plugin>`
-  Fixed lifecycle work directory cleaning
-  Made stage package tracking optional
-  Improved missing local source error message
-  Allow plus symbol in *git* url scheme
-  Add plain file source handler

.. _build-providers-1:

Build providers
---------------

-  Disable automatic snap refreshes inside instances
-  LXD instances launch from a cached base instance rather than a base image. This reduces disk usage and launch time.
-  For the LXD launch function launched_environment, the parameter use_snapshots has been replaced by use_base_instance. use_snapshots still works but logs a deprecation notice.
-  Expire and recreate base instances older than 3 months (90 days)
-  Check for network connectivity after network-related commands fails
-  Set LXD id maps after launching or copying an instance
-  Raise BaseConfigurationError for snap refresh failures
-  Check LXD id map before starting an existing instance. If the id map does not match, the instance will be auto cleaned or an error will be raised.

Metadata
--------

-  Added support for new system usernames `#3964 <https://github.com/snapcore/snapcraft/pull/3964>`__
-  Added support for top-level ``provenance`` keyword for on-prem support `#3963 <https://github.com/snapcore/snapcraft/pull/3963>`__

Installation
------------

-  Added *remove* hook to delete base instances and base images `#4014 <https://github.com/snapcore/snapcraft/pull/4014>`__

Linters
-------

-  Added linter check for unused libraries `#4028 <https://github.com/snapcore/snapcraft/pull/4028>`__ and help urls `#3954 <https://github.com/snapcore/snapcraft/pull/3954>`__

Command line interface
----------------------

-  Set default verbosity level with environment variable `#3958 <https://github.com/snapcore/snapcraft/pull/3958>`__
-  Accept snap file in legacy upload-metadata command `#3975 <https://github.com/snapcore/snapcraft/pull/3975>`__
-  Pass ``--verbose`` and ``-v`` to snapcraft_legacy `#4024 <https://github.com/snapcore/snapcraft/pull/4024>`__
-  Improved *docstring* for ``get_build_provider_flags`` `#4025 <https://github.com/snapcore/snapcraft/pull/4025>`__
-  Fixed ``StoreLegacyRegisterKeyCommand`` overview `#3984 <https://github.com/snapcore/snapcraft/pull/3984>`__
-  :ref:`snapcraft try <iterating-over-a-build>` for core22 `#3981 <https://github.com/snapcore/snapcraft/pull/3981>`__

Classic
-------

-  Patched elf files for classic mode `#3985 <https://github.com/snapcore/snapcraft/pull/3985>`__
