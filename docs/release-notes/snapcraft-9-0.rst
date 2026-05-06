.. meta::
    :description: Learn about the new features, changes, and fixes introduced in Snapcraft 9.0.

.. _release-9.0:

Snapcraft 9.0 release notes
===========================

7 May 2026

Learn about the new features, changes, and fixes introduced in Snapcraft 9.0.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 9.0 brings the following features, integrations, and improvements.


Support for core26
~~~~~~~~~~~~~~~~~~

Snapcraft now supports building snaps with the core26 base. Core26 is stable and
recommended for all new snaps.

If you're currently using core24, you can migrate by following the
:ref:`how-to-change-from-core24-to-core26` guide.

If you were already building experimental core26 snaps, you can drop
``build-base: devel`` or ``grade: devel`` from your project file. As a result, your
snaps will be eligible for publication to candidate and stable channels in the Snap
Store.

GPU extension
~~~~~~~~~~~~~

Snaps support hardware-accelerated graphics like OpenGL and Vulkan, but accessing GPU
libraries usually requires a full desktop extension, increasing complexity and build
time.

A :ref:`reference-gpu-extension` has been added for core22 and higher snaps, which
provides hardware-accelerated graphics support for apps. If your snap used a desktop
extension like GNOME as a workaround for accessing GPU capabilities, replace it with
this new extension.

GPU linter
~~~~~~~~~~

A new linter has been added for snaps that need GPU libraries.
If a snap stages GPU libraries with the :ref:`stage-packages <PartSpec.stage_packages>`
key, the linter suggests using a GPU content snap instead.

The :ref:`how-to-use-the-gpu-linter` guide describes how to address issues flagged by
the linter.

Stable .NET extensions
~~~~~~~~~~~~~~~~~~~~~~

The :ref:`reference-dotnet-extensions` are now stable and no longer require
the ``SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS`` environment variable.

Bazel plugin
~~~~~~~~~~~~

The :ref:`craft_parts_bazel_plugin` is now available for parts that need the Bazel
build system.

npm Use plugin
~~~~~~~~~~~~~~

A :ref:`craft_parts_npm_use_plugin` has been added. This plugin exports npm
tarballs to a local directory.

Additionally, the :ref:`craft_parts_npm_plugin` plugin gained support for the
``self-contained`` build attribute.

Together, these changes enable npm parts to build from local sources.

Maven Use plugin
~~~~~~~~~~~~~~~~

A :ref:`craft_parts_maven_use_plugin` has been added. This plugin deploys Maven
artifacts to a local repository, enabling other Maven parts to build from local
sources.


Minor features
--------------

Snapcraft 9.0 brings the following minor changes.

Promote edge channels with ``--yes``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`ref_commands_promote` command now supports promoting releases from the edge
channel non-interactively with the ``--yes`` flag.

``--format`` option for ``validation-sets``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`ref_commands_validation-sets` command now supports a ``--format`` option to
output validation sets as either a table or JSON.

Interactive key selection for ``register-key``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`ref_commands_register-key` command now prompts you to select a key
when no key name is given.

Progressive percentage shown after ``release``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`ref_commands_release` command now reports the progressive percentage
when releasing a snap progressively. For example::

    Released 'my-snap' revision 42 to channels: 'stable' for 30% of users

No linting of donation links
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`metadata linter <how-to-use-the-metadata-linter>` no longer checks for the
:ref:`Project.donation` key, as this key was generally rarely set and was silenced more
often than not.

Gradle plugin improvements
~~~~~~~~~~~~~~~~~~~~~~~~~~

The Gradle daemon is now disabled by default when using the
:ref:`craft_parts_gradle_plugin`. You can control this behavior with the new
``gradle-use-daemon`` key.

The plugin also supports the ``self-contained`` build attribute, so that parts using the
:ref:`craft_parts_maven_use_plugin` can use local dependencies.

7zip support
~~~~~~~~~~~~

Parts can now source 7zip files ending in either ``.7zip`` or ``.7z``. Previously, 7zip
files had to end in ``.7zip``.

Additionally, 7zip files are now documented in the :ref:`source-type <PartSpec.source_type>`
key in the project file reference.

Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Removed core20 support
~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 9 removes support for building core20 snaps. Use Snapcraft 8 to continue
building core20 snaps.

The :ref:`support schedule <reference-support-schedule>` details our remaining
commitments to core20.

Renamed commands
~~~~~~~~~~~~~~~~

We strive to improve the clarity and simplicity of the Snapcraft CLI. Over several
versions of Snapcraft 8, we revised many commands and provided deprecation warnings for
their old verbs.

Snapcraft 9 removes the old command names. The updated names are:

.. list-table::
    :header-rows: 1
    :widths: 1 2

    * - Old command
      - New command
    * - ``list``
      - ``names``
    * - ``list-registered``
      - ``names``
    * - ``list-extensions``
      - ``extensions``
    * - ``list-plugins``
      - ``plugins``
    * - ``list-tracks``
      - ``tracks``
    * - ``list-revisions``
      - ``revisions``
    * - ``list-validation-sets``
      - ``validation-sets``
    * - ``list-confdb-schemas``
      - ``confdb-schemas``
    * - ``list-keys``
      - ``keys``
    * - ``push``
      - ``upload``
    * - ``snap``
      - ``pack``


Additionally, Snapcraft 9 removes the following command line options. They're replaced
by their equivalent environment variables:

.. list-table::
    :header-rows: 1
    :widths: 2 3 4

    * - Command
      - Old option
      - Replacement
    * - ``export-login``
      - ``-experimental-login``
      - Set the environment variable ``SNAPCRAFT_STORE_AUTH=candid``.
    * - ``login``
      - ``-experimental-login``
      - Set the environment variable ``SNAPCRAFT_STORE_AUTH=candid``.
    * - ``login``
      - ``-with``
      - Export the credentials to the environment variable ``SNAPCRAFT_STORE_CREDENTIALS``.


Removed legacy remote builder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The legacy remote builder has been removed in favor of the current :ref:`remote builder
<explanation-remote-build>`.

The environment variable to invoke the legacy remote builder,
``SNAPCRAFT_REMOTE_BUILD_STRATEGY``, is no longer used by Snapcraft.

Previously, if Snapcraft couldn't find remote build credentials,
it would try to load credentials from the legacy location
``$XDG_DATA_DIR/snapcraft/provider/launchpad/credentials``.

Snapcraft 9.0 only loads credentials from
``$XDG_DATA_DIR/snapcraft/launchpad-credentials``, and doesn't use the fallback.


Removed snapcraftctl for core26
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 8 deprecated the ``snapcraftctl`` command in override scripts for core22 and
core24 snaps in favor of :ref:`craftctl <reference-external-package-scriptlets>`.

Core26 snaps only support ``craftctl``. To use core26, you must replace all
instances of ``snapcraftctl`` in your scripts.

Core22 and core24 aren't affected by this change.

Removed forward slashes in core26 part names
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Core26 snaps can no longer use forward slashes (/) in part names. The
:ref:`migration guide <how-to-change-from-core24-to-core26>` describes how to update
parts with forward slashes in their name.

Removed support for Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft previously supported running on Windows. This workflow was uncommon,
because snaps aren't compatible with Windows.

Snapcraft 9 drops all official support for Windows. To build snaps on Windows
technology, use Windows Subsystem for Linux (WSL) as described in
:ref:`how-to-set-up-snapcraft`.

Updated documentation system
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The documentation base is updated to Canonical's Sphinx Starter Pack 1.4.0. Going
forward, the system will stay in step with the Starter Pack, keeping pace with its
features.

The documentation commands that are standard in Canonical products are accessible by
prefixing them with ``docs-``:

.. list-table::
    :header-rows: 1
    :widths: 1 4

    * - Command
      - Result
    * - ``make docs``
      - Renders the docs as a static set of HTML pages.
    * - ``make docs-auto``
      - Hosts the docs in a local server you can view in the web browser. When you save
        a change to a source file, the server updates the doc in real time.
    * - ``make docs-lint``
      - Checks for problems in the documentation.
    * - ``make docs-clean``
      - Removes the built docs and temporary files.
    * - ``make docs-help``
      - See the full list of commands from the Starter Pack.

The Starter Pack is no longer a Git submodule. If you've written for Snapcraft 8 or
lower, or built the documentation before, you must remove the submodule from your host to
continue developing:

.. code-block:: bash

    git submodule deinit -f docs/sphinx-docs-starter-pack
    rm -r docs/sphinx-docs-starter-pack


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 9.0.

.. _release-notes-fixes-9.0.0:

Snapcraft 9.0.0
~~~~~~~~~~~~~~~~

- `#6122 <https://github.com/canonical/snapcraft/issues/6122>`__ Content interface mount
  targets weren't created during build for snaps using base core26+ or bare.
- `#6054 <https://github.com/canonical/snapcraft/issues/6054>`__ The GNOME
  extension was missing symlink to libproxy.
- `#6148 <https://github.com/canonical/snapcraft/issues/6148>`__ Dependencies in the
  core26 snap were included when staging packages.
- `craft-parts#1444 <https://github.com/canonical/craft-parts/issues/1444>`__ Stage and
  build packages with versioned dependencies couldn't be resolved.
- `craft-parts#1492 <https://github.com/canonical/craft-parts/issues/1492>`__ The
  :ref:`craft_parts_poetry_plugin` didn't work for core26 snaps.


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.


:literalref:`@Amanlem <https://github.com/Amanlem>`,
:literalref:`@asanvaq <https://github.com/asanvaq>`,
:literalref:`@atandrewlee <https://github.com/atandrewlee>`,
:literalref:`@bboozzoo <https://github.com/bboozzoo>`,
:literalref:`@bepri <https://github.com/bepri>`,
:literalref:`@brlin-tw <https://github.com/brlin-tw>`,
:literalref:`@canon-cat <https://github.com/canon-cat>`,
:literalref:`@cmatsuoka <https://github.com/cmatsuoka>`,
:literalref:`@EddyPronk <https://github.com/EddyPronk>`,
:literalref:`@ethandcosta <https://github.com/ethandcosta>`,
:literalref:`@gcomneno <https://github.com/gcomneno>`,
:literalref:`@Guillaumebeuzeboc <https://github.com/Guillaumebeuzeboc>`,
:literalref:`@jahn-junior <https://github.com/jahn-junior>`,
:literalref:`@jawadsalwati <https://github.com/jawadsalwati>`,
:literalref:`@lengau <https://github.com/lengau>`,
:literalref:`@mateusrodrigues <https://github.com/mateusrodrigues>`,
:literalref:`@mbeijen <https://github.com/mbeijen>`,
:literalref:`@medubelko <https://github.com/medubelko>`,
:literalref:`@mr-cal <https://github.com/mr-cal>`,
:literalref:`@Namrathabp <https://github.com/Namrathabp>`,
:literalref:`@PraaneshSelvaraj <https://github.com/PraaneshSelvaraj>`,
:literalref:`@Saviq <https://github.com/Saviq>`,
:literalref:`@smethnani <https://github.com/smethnani>`,
:literalref:`@steinbro <https://github.com/steinbro>`,
:literalref:`@tigarmo <https://github.com/tigarmo>`,
:literalref:`@toroleapinc <https://github.com/toroleapinc>`,
and :literalref:`@vedantdaterao <https://github.com/vedantdaterao>`
