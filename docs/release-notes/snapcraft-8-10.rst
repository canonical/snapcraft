.. _release-8.10:

Snapcraft 8.10 release notes
============================

7 July 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.10.


Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.10 brings the following features, integrations, and improvements.


.NET plugin (v2)
~~~~~~~~~~~~~~~~

The :ref:`craft_parts_dotnet_v2_plugin` is now available for core24 snaps. We've
introduced a new version of the plugin that is easier to use and configure.

Older bases will continue to use the :ref:`previous version <craft_parts_dotnet_plugin>`
of the dotnet plugin.

Init profile for ROS 2 applications
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A new init profile, ``ros2``, has been added. This profile makes it easier for developers
to create an ROS 2 snap.


Improved ``test`` command
~~~~~~~~~~~~~~~~~~~~~~~~~

The ``CRAFT_ARTIFACT`` environment variable can now be used in tests and contains the
full path to the snap file being tested.

Additionally, the user experience of the ``test`` command has been cleaned up and
improved.


Minor features
--------------

Snapcraft 8.10 brings the following minor changes.


Support for CMake in the GNOME extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

CMake projects using the :ref:`reference-gnome-extension` are now
able to use libraries from the GNOME SDK snap with `find_library
<https://cmake.org/cmake/help/latest/command/find_library.html>`_.

During the build step, the GNOME extension now points the ``CMAKE_PREFIX_PATH``
environment variable to the GNOME SDK snap and the stage directory.

If a part using the :ref:`craft_parts_cmake_plugin` needs to override this variable,
it should be done with the ``build-environment`` key instead of ``cmake-parameters:
[-DCMAKE_PREFIX_PATH]``.


Summaries for confdb schemas
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``edit-confdb-schema`` command now supports a top-level
``summary`` key and a ``summary`` key in each view of a `confdb schema
<https://snapcraft.io/docs/configure-with-confdb>`_.


Validation set sequence warning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If a user forgets to increment the sequence when editing a `validation set
<https://snapcraft.io/docs/validation-sets>`_, it can prevent snaps from being able to
revert to a valid state.

The :ref:`ref_commands_edit-validation-sets` command will now warn the user if the
sequence hasn't been incremented and prompt them to re-edit the set before submitting
it.


Documentation feedback button
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We've added a feedback button to the top of all pages to make it easier to report issues
with the documentation.


Updated guidance for setting up Snapcraft
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`how-to-set-up-snapcraft` page has been rewritten based on user feedback and
testing.


Scheduled feature deprecations
------------------------------

The following features will be deprecated in a future major release:


Old command aliases
~~~~~~~~~~~~~~~~~~~

Previous Snapcraft releases renamed commands, and aliased the old command names to
the new command names.

This release adds a deprecation warning when an old command name is used. The old
command names are still available, but will be removed in a future release.

.. list-table::
    :header-rows: 1

    * - Old command
      - New command
    * - ``list``
      - ``names``
    * - ``list-registered``
      - ``names``
    * - ``extensions``
      - ``list-extensions``
    * - ``plugins``
      - ``list-plugins``
    * - ``tracks``
      - ``list-tracks``
    * - ``revisions``
      - ``list-revisions``
    * - ``push``
      - ``upload``
    * - ``snap``
      - ``pack``


Legacy remote builder for core22 snaps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Support for using the legacy remote builder for core22 snaps will be dropped in a future
release.

Core22 snaps should use the current remote builder instead, which is the default
behavior. For more information, see :ref:`explanation-remote-build`.


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.10.


Snapcraft 8.10.0
~~~~~~~~~~~~~~~~

- `#5161`_ Invalid text in the ``SNAPCRAFT_STORE_CREDENTIALS`` environment variable now
  emits an error instead of a traceback.

- `#5167`_ If a file called ``snap`` exists in the project directory, Snapcraft now
  emits an error instead of a traceback.

- `#5539`_ Snapcraft no longer emits internal deprecation warnings when running on
  Python 3.13.

- ``CRAFT_PARALLEL_BUILD_COUNT`` and ``CRAFT_MAX_PARALLEL_BUILD_COUNT`` are now
  forwarded to managed instances.


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@artivis<https://github.com/artivis>`,
:literalref:`@astrojuanlu<https://github.com/astrojuanlu>`,
:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@cmatsuoka<https://github.com/cmatsuoka>`,
:literalref:`@edisile<https://github.com/edisile>`,
:literalref:`@j-g00da<https://github.com/j-g00da>`,
:literalref:`@jahn-junior<https://github.com/jahn-junior>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`,
:literalref:`@lengau<https://github.com/lengau>`,
:literalref:`@nandedamana<https://github.com/nandedamana>`,
:literalref:`@panagiotisevaggelou<https://github.com/panagiotisevaggelou>`, and
:literalref:`@pedro-avalos<https://github.com/pedro-avalos>`

.. _#5161: https://github.com/canonical/snapcraft/issues/5161
.. _#5167: https://github.com/canonical/snapcraft/issues/5167
.. _#5539: https://github.com/canonical/snapcraft/issues/5539
