.. meta::
    :description: Learn about the new features, changes, and fixes introduced in Snapcraft 9.1.

.. _release-9.1:

Snapcraft 9.1 release notes
===========================

08 September 2026

Learn about the new features, changes, and fixes introduced in Snapcraft 9.1.


Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 9.1 brings the following features, integrations, and improvements.

Support for Ubuntu Pro-compliant snaps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 9.1 supports packing snaps containing the extended security maintenance fixes
and compliance features of Ubuntu Pro. System requirements and guidance on packing
Pro-compliant snaps can be found in :ref:`how-to-pack-a-pro-snap`.

Kernel and initrd plugins
~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`reference-kernel-plugin` has been reworked and is no longer experimental.

A companion :ref:`reference-initrd-plugin` has also been added for building the
initial ramdisk used by a kernel snap.

Experimental ROS 2 Lyrical extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The experimental :ref:`ros2-lyrical extension <reference-ros-2-extensions>` has been
added for core26 snaps, joining the existing ros2-humble and ros2-jazzy extensions for
core22 and core24, respectively.

Experimental .NET 11 extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The experimental :ref:`dotnet11 extension <reference-dotnet-extensions>` has been added
for apps targeting .NET 11.

Support for armv8l
~~~~~~~~~~~~~~~~~~

64-bit ARM systems can now natively build ARMHF snaps by setting the environment
variable ``CRAFT_BUILD_ON=armhf``. Snapcraft recognizes the armv8l machine type,
reported by these systems when running a 32-bit userspace, and maps it to ARMHF.


Minor features
--------------

Snapcraft 9.1 brings the following minor changes.

Conditional repacking
~~~~~~~~~~~~~~~~~~~~~

To speed up redundant ``snapcraft pack`` commands, Snapcraft now only repacks snaps
if the source changed or the snap's metadata need to be updated. This is useful when
iterating with ``snapcraft test``, as unchanged snaps aren't rebuilt between test runs.

Per-component compression
~~~~~~~~~~~~~~~~~~~~~~~~~

Components can now set their own :ref:`compression <component.compression>` in the
project file, overriding the compression algorithm used for the snap itself. This
is useful when a component's payload is already heavily compressed, such as a large
language model's weights. Compressing it again with the snap's default algorithm
increases decompression time at runtime without meaningfully reducing its size.

Agents file
~~~~~~~~~~~

An `AGENTS.md <https://github.com/canonical/snapcraft/blob/main/AGENTS.md>`__ file has
been added to the root of the repository, describing the codebase and practices for
developers using AI coding agents to assist with their contributions to Snapcraft.

Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Removed Matter SDK plugin
~~~~~~~~~~~~~~~~~~~~~~~~~

The experimental Matter SDK plugin has been removed. It was originally added to
simplify building the `Matter SDK <https://github.com/project-chip/connectedhomeip>`__,
which required patches to the upstream source code. Those patches have since been
merged into the Matter project, so the plugin is no longer needed. If
your snap used the Matter SDK plugin, follow `Matter's compilation guide
<https://project-chip.github.io/connectedhomeip-doc/getting_started/first_example.html#compiling-running-and-controlling-matter-examples>`__
as done in the
`Chip Tool <https://github.com/canonical/chip-tool-snap/blob/main/snap/snapcraft.yaml>`__
snap.

Removed Candid login
~~~~~~~~~~~~~~~~~~~~

Candid, a deprecated authentication method for the Snap Store, has been removed.
Instead, :ref:`authenticate with Ubuntu One <how-to-authenticate>`.


Feature deprecations
--------------------

The following features are deprecated in Snapcraft 9.1:

``ua-services`` key for core24 and higher
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`Project.ua_services` key was unused for core24 and higher snaps. It's now been
deprecated in favor of the new ``--pro=<services>`` command-line option described in
:ref:`how-to-pack-a-pro-snap`.

Non-SPDX compliant licenses
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Setting the :ref:`Project.license` key to a value that isn't a valid `SPDX license
expression <https://spdx.org/licenses/>`__ is now deprecated. Non-SPDX values are still
accepted, but won't be allowed in future bases.


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 9.1.

.. _release-notes-fixes-9.1.0:

Snapcraft 9.1.0
~~~~~~~~~~~~~~~

- `craft-application#1073 <https://github.com/canonical/craft-application/pull/1073>`__
  ``--debug`` wouldn't enter a debug shell for failures after the prime step, such as
  a failure to run ``snap pack``.
- `craft-archives#229
  <https://github.com/canonical/craft-archives/pull/229>`__ When retrieving the signing
  key for a package repository, GPG wouldn't use the system's proxy.
- `craft-cli#449 <https://github.com/canonical/craft-cli/issues/449>`__ Deprecated
  commands appeared in the *See also* section of command help text.
- `craft-parts#1396 <https://github.com/canonical/craft-parts/issues/1396>`__ Certain
  network errors while downloading file sources produced a raw traceback instead of a
  clear error message.
- `craft-parts#1562 <https://github.com/canonical/craft-parts/issues/1562>`__ The
  :ref:`organize <PartSpec.organize_files>` key allowed organizing files from outside
  the install directory.
- `snapcraft#6327 <https://github.com/canonical/snapcraft/issues/6327>`__ Desktop
  extensions incorrectly set ``LD_LIBRARY_PATH``, preventing Snapcraft from setting
  its own default value and breaking library loading.


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@asanvaq <https://github.com/asanvaq>`,
:literalref:`@bepri <https://github.com/bepri>`,
:literalref:`@canon-cat <https://github.com/canon-cat>`,
:literalref:`@cmatsuoka <https://github.com/cmatsuoka>`,
:literalref:`@dilyn-corner <https://github.com/dilyn-corner>`,
:literalref:`@EdmilsonRodrigues <https://github.com/EdmilsonRodrigues>`,
:literalref:`@elijahgreenstein <https://github.com/elijahgreenstein>`,
:literalref:`@florcabral <https://github.com/florcabral>`,
:literalref:`@gcomneno <https://github.com/gcomneno>`,
:literalref:`@giusebar <https://github.com/giusebar>`,
:literalref:`@imatrisciano <https://github.com/imatrisciano>`,
:literalref:`@jahn-junior <https://github.com/jahn-junior>`,
:literalref:`@kubiko <https://github.com/kubiko>`,
:literalref:`@lengau <https://github.com/lengau>`,
:literalref:`@mateusrodrigues <https://github.com/mateusrodrigues>`,
:literalref:`@medubelko <https://github.com/medubelko>`,
:literalref:`@MirkoFerrati <https://github.com/MirkoFerrati>`,
:literalref:`@Mohit-Chachada <https://github.com/Mohit-Chachada>`,
:literalref:`@mr-cal <https://github.com/mr-cal>`,
:literalref:`@PraaneshSelvaraj <https://github.com/PraaneshSelvaraj>`,
:literalref:`@shaloo <https://github.com/shaloo>`,
:literalref:`@smethnani <https://github.com/smethnani>`,
:literalref:`@soumyaDghosh <https://github.com/soumyaDghosh>`,
:literalref:`@steinbro <https://github.com/steinbro>`,
:literalref:`@Tejas-Raj01 <https://github.com/Tejas-Raj01>`,
:literalref:`@tigarmo <https://github.com/tigarmo>`,
:literalref:`@upils <https://github.com/upils>`,
and :literalref:`@zhijie-yang <https://github.com/zhijie-yang>`
