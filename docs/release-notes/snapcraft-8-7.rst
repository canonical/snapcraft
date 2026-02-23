.. _release-8.7:

Snapcraft 8.7 release notes
=============================

03 March 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.7.


Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.7 brings the following features, integrations, and improvements.


Bash completion
~~~~~~~~~~~~~~~

Previously, the completion file for Snapcraft was outdated or incorrect because
it had to be manually updated.

We've updated the completion file to be generated dynamically, which means it will
always autocomplete the latest commands and options in Bash-compatible shells.
Try it out by typing ``snapcraft`` and pressing :kbd:`Tab` in your terminal.


Improved remote builder
~~~~~~~~~~~~~~~~~~~~~~~

Remote builds can now use the ``--build-for`` option to filter entries in an
``architectures`` or ``platforms`` key in a project file.


Support for confdbs
~~~~~~~~~~~~~~~~~~~

A `confdb schema <https://snapcraft.io/docs/configure-with-confdb>`_ defines the
configuration of Linux systems, including storage, access permission, granularity,
and sharing between snaps.

Snapcraft now supports listing and editing ``confdbs`` with the commands
``list-confdbs`` and ``edit-confdbs``. These new commands replace the previous
``list-registries`` and ``edit-registries`` commands, respectively.


Support for relocatable pkgconfig files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now checks if `pkg-config`_ (``.pc``) files are relocatable. If the file is
relocatable, Snapcraft will no longer modify its ``prefix`` field.


Documentation improvements
~~~~~~~~~~~~~~~~~~~~~~~~~~

The following how-to guides have been integrated:

* :ref:`Use an extension <how-to-use-an-extension>`
* :ref:`List extensions <how-to-list-extensions>`
* :ref:`Enable experimental extensions <how-to-enable-experimental-extensions>`
* :ref:`Use the env-injector extension <how-to-use-the-env-injector-extension>`
* :ref:`Use the GNOME extension <how-to-use-the-gnome-extension>`


The following references have been integrated:

* :ref:`env-injector extension <reference-env-injector-extension>`
* :ref:`GNOME extension <reference-gnome-extension>`
* :ref:`KDE neon extensions <reference-kde-neon-extensions>`
* :ref:`Flutter extension <reference-flutter-extension>`
* :ref:`ROS 1 extension <reference-ros-1-extension>`
* :ref:`ROS 1 content extensions <reference-ros-1-content-extensions>`
* :ref:`ROS 2 extensions <reference-ros-2-foxy-extension>`
* :ref:`ROS 2 content extensions <reference-ros-2-content-extensions>`


Backwards-incompatible changes
------------------------------

Removed platform option for remote builds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``--platform`` option has been removed from the ``remote-build`` command.

This option was dropped because it doesn't provide predictable results for remote
builds, due to Launchpad's handling of the ``platforms`` key in project files.

``--build-for`` is the recommended alternative until Launchpad has comprehensive
support for platforms.

Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.7:

8.7.0
~~~~~

- `#5250`_ Resources path for ``QtWebEngineProcess`` wasn't exported for snaps
  using the KDE Neon 6 extension.
- `craft-parts#978`_ The ``source-subdir`` field was ignored for the
  :ref:`Go Use plugin<craft_parts_go_use_plugin>`.
- `craft-application#600`_ The same build environment may be re-used for platforms with
  the same ``build-on`` and ``build-for`` architectures.
- `craft-application#618`_ The remote builder would clean up projects after
  they timed out.
- `craft-application#619`_ The remote builder suggested using a nonexistent
  ``--build-id`` option if the build timed out.
- `craft-application#620`_ The remote builder help suggested using a nonexistent
  ``--status`` option.

.. _release-notes-fixes-8.7.1:

8.7.1
~~~~~

- `#5258`_ The Flutter plugin failed to install Flutter for ``core22`` and ``core24``
  snaps.

.. _release-notes-fixes-8.7.2:

8.7.2
~~~~~

- `craft-parts#991`_ Classic snaps using the
  :ref:`uv plugin<craft_parts_uv_plugin>` would fail to find the Python
  interpreter included in the snap itself.

.. _release-notes-fixes-8.7.3:

8.7.3
~~~~~

- `#5340`_ Always show deprecation warnings for ``snapcraft list`` and
  ``snapcraft list-registered`` commands.
- `craft-parts#1025`_ The final lines of stdout or stderr when building a part
  may not be logged.

.. _release-notes-fixes-8.7.4:

8.7.4
~~~~~

- `#5270`_ The remote-builder gave an unfriendly error when using the
  ``--build-for`` argument and shorthand :doc:`platforms </reference/architectures>`
  entries in the project file.
- `#5330`_ The Crystal plugin would fail to properly quote environment variables.

Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`,
and :literalref:`@sergio-costas<https://github.com/sergio-costas>`

.. _#5250: https://github.com/canonical/snapcraft/pull/5250
.. _#5270: https://github.com/canonical/snapcraft/pull/5270
.. _#5258: https://github.com/canonical/snapcraft/pull/5258
.. _#5340: https://github.com/canonical/snapcraft/pull/5340
.. _#5330: https://github.com/canonical/snapcraft/issues/5330
.. _craft-application#600: https://github.com/canonical/craft-application/issues/600
.. _craft-application#618: https://github.com/canonical/craft-application/issues/618
.. _craft-application#619: https://github.com/canonical/craft-application/issues/619
.. _craft-application#620: https://github.com/canonical/craft-application/issues/620
.. _craft-parts#978: https://github.com/canonical/craft-parts/issues/978
.. _craft-parts#991: https://github.com/canonical/craft-parts/issues/991
.. _craft-parts#1025: https://github.com/canonical/craft-parts/issues/1025
.. _pkg-config: https://www.freedesktop.org/wiki/Software/pkg-config/
