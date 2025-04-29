.. _release-8.9:

Snapcraft 8.9 release notes
===========================

12 May 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.9.

Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.

What's new
----------

Snapcraft 8.9 brings the following features, integrations, and improvements.

``snapcraft test``
~~~~~~~~~~~~~~~~~~

A new command called ``test``, and its accompanying ``init`` profile, are available for
testing snaps. They leverage `Spread`_ to run identically on local and remote systems.

The command is experimental and subject to change. To try it on an existing snap
project, run ``snapcraft init --profile=test`` at the root of your project directory.
For a description of the command and its options, see the :ref:`test command
<ref_commands_test>` reference.

Cargo use plugin
~~~~~~~~~~~~~~~~

We added a plugin that sets up a local `cargo registry`_ for `Rust`_ crates. These
crates can then be used by the existing :ref:`Rust plugin <craft_parts_rust_plugin>`.
See the :ref:`craft_parts_cargo_use_plugin` reference for details.

Gradle plugin
~~~~~~~~~~~~~

Previously, Snapcraft supported the `Gradle`_ build tool for core18 snaps.

Support for Gradle is now available for core22 and core24 snaps with a new Gradle
plugin. See the :ref:`craft_parts_gradle_plugin` reference for details.

Documentation migration and redesign
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, Snapcraft's documentation was hosted on `snapcraft.io
<http://snapcraft.io/docs>`_ and edited via forum posts. The documentation has been
migrated to :doc:`Read the Docs </index>` and is hosted in Snapcraft's repository.

The documentation was redesigned to follow `Diátaxis <https://diataxis.fr/>`_.
It has a similar design to documentation from other Canonical products and
is versioned and released alongside Snapcraft itself.

Minor features
--------------

Snapcraft 8.9 brings the following minor changes.

Support for Maven wrappers
~~~~~~~~~~~~~~~~~~~~~~~~~~

The Maven plugin now supports the ``maven-use-wrapper`` key to indicate that the build
should use the wrapper provided by the source code. See the
:ref:`craft_parts_maven_plugin` reference for details.

Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Renaming of confdb schema commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A confdb schema defines the configuration of Linux systems, including storage,
access permission, granularity, and sharing between snaps.

The existing confdb schema commands have been renamed. The commands
``list-confdb-schemas`` and ``edit-confdb-schema`` replace the previous
``list-confdbs`` and ``edit-confdbs`` commands, respectively.

Known issues
------------

The following issues were reported and are scheduled to be fixed in upcoming
patch releases.

See individual issue links for any mitigations.

- `#5272`_ The GNOME extension sets the wrong ``CMAKE_PREFIX_PATH``.

Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.9.

Snapcraft 8.9.0
~~~~~~~~~~~~~~~

- `#5107`_ Plugin errors would provide broken links to documentation.

Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@cmatsuoka<https://github.com/cmatsuoka>`,
:literalref:`@jahn-junior<https://github.com/jahn-junior>`,
:literalref:`@lengau<https://github.com/lengau>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`,
:literalref:`@sergiusens<https://github.com/sergiusens>`,
:literalref:`@soumyaDghosh<https://github.com/soumyaDghosh>`,
:literalref:`@st3v3nmw<https://github.com/st3v3nmw>`,
and :literalref:`@tigarmo<https://github.com/tigarmo>`

.. _Gradle: https://gradle.org/
.. _Rust: https://doc.rust-lang.org/stable/
.. _Spread: https://github.com/snapcore/spread
.. _cargo registry: https://doc.rust-lang.org/cargo/reference/registries.html
.. _#5107: https://github.com/canonical/snapcraft/pull/5107
.. _#5272: https://github.com/canonical/snapcraft/pull/5272
