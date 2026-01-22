.. _release-8.14:

Snapcraft 8.14 release notes
============================

03 Feburary 2026

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

A new :ref:`craft_parts_ruby_plugin` is available for building Ruby apps for core22 and
newer snaps.

JLink options
~~~~~~~~~~~~~

The :ref:`craft_parts_jlink_plugin` has new options, ``jlink-multi-release`` and
``jlink-modules``.


``success-exit-status`` key
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Daemons can use the new :ref:`success-exit-status <App.success_exit_status>` key to
control exit status handling.

KDE neon 6 theming
~~~~~~~~~~~~~~~~~~

The :ref:`KDE neon 6 <reference-kde-neon-extensions>` extension now supports LXQt and
Kvantum theming.


Minor features
--------------

Snapcraft 8.14 brings the following minor changes.

Chisel support for unstable releases
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Chisel slices can now be used for :ref:`stage-packages <PartSpec.stage_packages>` for
core26 snaps and snaps using ``build-base: devel``.

Collision detection for ``organize``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now checks for collisions when organizing with the :ref:`organize
<PartSpec.organize_files>` key. Collisions will now raise an error.

Destructive mode warning
~~~~~~~~~~~~~~~~~~~~~~~~

Users are now warned when they run Snapcraft in destructive mode as a non-root user,
which can cause potentially unexpected behavior.

Git improvements
~~~~~~~~~~~~~~~~

A number of improvements have been made for parts using the ``git``
:ref:`source-type <PartSpec.source_type>`.

- Shallow clones of git sources are now possible when using :ref:`source-commit
  <PartSpec.source_commit>` with :ref:`source-depth <PartSpec.source_depth>`.
- When cloning Git repos, detached HEAD warnings are now suppressed.
- Running ``git describe --dirty`` in an override script of a clean repository no longer
  falsely reports the repository as dirty.

Improved project file schema
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IDEs use Snapcraft’s schema to validate and auto-complete ``snapcraft.yaml`` files. The
schema has been reworked to improve validation and properly support base-specific keys.

The schema is now automatically generated, so it will stay up-to-date as
new keys are added.

uv plugin bytecode
~~~~~~~~~~~~~~~~~~

The :ref:`craft_parts_uv_plugin` now compiles bytecode. Use ``UV_COMPILE_BYTECODE=0``
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
