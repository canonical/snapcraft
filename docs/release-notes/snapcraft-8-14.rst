.. _release-8.14:

Snapcraft 8.14 release notes
============================

03 February 2026

Learn about the new features, changes, and fixes introduced in Snapcraft 8.14.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.

What's new
----------

Snapcraft 8.14 brings the following features, integrations, and improvements.

Ruby plugin
~~~~~~~~~~~

The new :ref:`craft_parts_ruby_plugin` is available for packaging Ruby apps with
core22 and higher.

Improved project file schema
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IDEs read the Snapcraft schema to validate and auto-complete ``snapcraft.yaml`` files.

The schema is reworked to improve validation and support base-specific keys. It's also
automatically generated with each new release, so it will stay up-to-date as new keys
are added in new Snapcraft versions.


Minor features
--------------

Snapcraft 8.14 brings the following minor changes.

Chisel support for core26
~~~~~~~~~~~~~~~~~~~~~~~~~

Chisel slices can now be used for :ref:`stage-packages <PartSpec.stage_packages>` for
core26 snaps.

Collision detection for ``organize``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now checks for collisions when organizing with the :ref:`organize
<PartSpec.organize_files>` key. Collisions will now raise an error.

Destructive mode warning
~~~~~~~~~~~~~~~~~~~~~~~~

Running Snapcraft in destructive mode with a non-root user can cause unexpected
behavior. Snapcraft now emits a warning if it's run this way.

Git source improvements
~~~~~~~~~~~~~~~~~~~~~~~

A number of improvements have been made for parts using the ``git``
:ref:`source-type <PartSpec.source_type>` for a part.

- You can make a shallow clone of the Git source with the new :ref:`source-depth
  <PartSpec.source_depth>` key.
- When cloning a Git source, detached HEAD warnings are now suppressed.

New JLink plugin keys
~~~~~~~~~~~~~~~~~~~~~

The :ref:`craft_parts_jlink_plugin` has new options, ``jlink-multi-release`` and
``jlink-modules``.

- The ``jlink-multi-release`` key specifies the OpenJDK release version to use for
  multi-release JARs.
- The ``jlink-modules`` key specifies the modules to include in the
  OpenJDK image.

LXQt support for KDE neon 6
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`KDE neon 6 <reference-kde-neon-extensions>` extension now supports LXQt and
Kvantum theming via the ``lxqt-support-core24`` content snap. Now, snaps using this
extension will render correctly in LXQt environments.

``success-exit-status`` key
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Daemons can use the new :ref:`success-exit-status <App.success_exit_status>` key to
control exit status handling.

uv plugin bytecode
~~~~~~~~~~~~~~~~~~

The :ref:`craft_parts_uv_plugin` now compiles bytecode. Set ``UV_COMPILE_BYTECODE=0``
to disable this feature.

Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Restrictions to platform names
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:ref:`Platform <reference-snapcraft-yaml-platform-keys>` names can't contain the ``/``
character. Additionally, platforms can't be named ``*`` or ``any``.


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.14.

Snapcraft 8.14.0
~~~~~~~~~~~~~~~~

- `craft-parts#1346`_ The Meson plugin now calls ``meson setup`` during build.

.. _release-notes-fixes-8.14.1:

Snapcraft 8.14.1
~~~~~~~~~~~~~~~~

- `craft-providers#898`_ Snapcraft retries calls to snapd when setting up the build
  environment.

Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@EdmilsonRodrigues<https://github.com/EdmilsonRodrigues>`,
:literalref:`@gabelluardo<https://github.com/gabelluardo>`,
:literalref:`@jahn-junior<https://github.com/jahn-junior>`,
:literalref:`@Kyuyrii<https://github.com/Kyuyrii>`,
:literalref:`@lengau<https://github.com/lengau>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@MirkoFerrati<https://github.com/MirkoFerrati>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`,
and :literalref:`@tigarmo<https://github.com/tigarmo>`.

.. _craft-parts#1346: https://github.com/canonical/craft-parts/pull/1346
.. _craft-providers#898: https://github.com/canonical/craft-providers/pull/898
