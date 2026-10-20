.. meta::
    :description: Learn about the new features, changes, and fixes introduced in Snapcraft 9.2.

.. _release-9.2:

Snapcraft 9.2 release notes
===========================

19 October 2026

Learn about the new features, changes, and fixes introduced in Snapcraft 9.2.


Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 9.2 brings the following features, integrations, and improvements.

GNOME extension support for core26
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`reference-gnome-extension` now supports core26 snaps. On core26, the
extension automatically prunes shared libraries already present in the GNOME and GTK
content snaps. This reduces the final snap size and prevents bundled dependencies from
conflicting with the content snaps.

Unified version selection with at signs (@)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In order to standardize package declarations, Snapcraft now supports the use of the at
sign (@) to separate a package name from its version. This sigil is supported for Deb
packages (``build-packages`` and ``stage-packages``) and snaps (``build-snaps`` and ``stage-snaps``).

The previous notation to declare package versions is still supported for all supported
bases up to core26, but future bases will only support the use of the character.

Init Git repositories
~~~~~~~~~~~~~~~~~~~~~

The :ref:`init command <ref_commands_init>` now features a ``--vcs`` option to
initialize a Git repository when creating the Snapcraft project.

stage-slices key
~~~~~~~~~~~~~~~~

Snapcraft 9.2 introduces the ``stage-slices`` key, which specifies a list of Chisel
slices to be cut into the part’s install directory. Slices can still be declared in
``stage-packages``, but this support will be dropped starting with base core28.

Support for confdb schema parameters and filters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now supports parameters and filters for confdb schemas. Parameters define
named constraints, which are used by filters to control the rules that apply in a confdb
schema. Parameter and filter definitions are outlined in the `confdb-schema reference
<https://documentation.ubuntu.com/core/reference/assertions/confdb-schema/>`__.


Minor features
--------------

Snapcraft 9.2 brings the following minor changes.

Always install phased APT updates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The phased rollout mechanism in APT was holding back package updates on some builds.
Snapcraft now configures APT to include phased packages to ensure that the latest
package version is installed.

build-base tracking for kernel snaps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Kernel snaps now record the ``build-base`` key in the snap’s metadata. This identifies
the build environment used to compile the kernel and helps snapd verify compatibility
with target systems.

Improved shebang handling in the Python plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the :ref:`python_plugin` on core26 could cause interpreter collisions between the
plugin’s virtual environment and the staged Python binary. Scripts resolving Python from
``PATH`` would bypass the virtual environment, causing import errors. The Python plugin
now changes the shebang path in scripts to ``/snap/<snap-name>/current/bin/python3`` to
ensure they use the correct interpreter.


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 9.2.

.. _release-notes-fixes-9.2.0:

Snapcraft 9.2.0
~~~~~~~~~~~~~~~

- `snapcraft#5583 <https://github.com/canonical/snapcraft/issues/5583>`__
  expand-extensions fails to process advanced grammar in parts/part/prime
- `snapcraft#5799 <https://github.com/canonical/snapcraft/issues/5799>`__ Valid desktop
  file without Exec key are rejected
- `snapcraft#5998 <https://github.com/canonical/snapcraft/issues/5998>`__
  ``primed-stage-packages`` in ``manifest.yaml`` loses architecture information
- `snapcraft#6093 <https://github.com/canonical/snapcraft/issues/6093>`__
  ``snapcraft init`` fails when the working directory's name is longer than 40 characters
- `snapcraft#6168 <https://github.com/canonical/snapcraft/issues/6168>`__ Can't
  "build-time resolve" staged symlinks with organize
- `snapcraft#6404 <https://github.com/canonical/snapcraft/issues/6404>`__ Library linter
  can give erroneous missing dependency warnings
- `snapcraft#6439 <https://github.com/canonical/snapcraft/issues/6439>`__ snapcraft
  metrics input validation fails when data range is provided but there is no data for
  certain dates
- `GHSL-2024-323 <https://securitylab.github.com/advisories/GHSL-2024-323_snapcraft/>`__
  Denial of Service (DoS) in snapcraft
- `LP#2083964 <https://bugs.launchpad.net/snapcraft/+bug/2083964>`__ 'dict' object has
  no attribute 'startswith' on ``expand-extensions`` on firefox nightly snap
- `craft-application#940 <https://github.com/canonical/craft-application/issues/940>`__
  The app crashes when loading YAML files that are not UTF-8 encoded
- `craft-application#1103 <https://github.com/canonical/craft-application/issues/1103>`__
  Remote build fails on dangling symlinks
- `craft-parts#292 <https://github.com/canonical/craft-parts/issues/292>`__
  Auto-populate apt-cache if empty
- `craft-parts#355 <https://github.com/canonical/craft-parts/issues/355>`__ Source
  checksum mismatch is confusing
- `craft-parts#507 <https://github.com/canonical/craft-parts/issues/507>`__ Permissions
  owner and group not applied
- `craft-parts#898 <https://github.com/canonical/craft-parts/issues/898>`__ Git source
  should check out a commit before recursing submodules
- `craft-parts#950 <https://github.com/canonical/craft-parts/issues/950>`__ Qmake plugin
  ignores source-subdir parameter
- `craft-parts#1218 <https://github.com/canonical/craft-parts/issues/1218>`__ "Skipping
  build for serde (already ran)" despite build-environment changing
- `craft-parts#1178 <https://github.com/canonical/craft-parts/issues/1178>`__ Show the
  URL of the package that could not be retrieved

.. _release-notes-fixes-9.2.1:

Snapcraft 9.2.1
~~~~~~~~~~~~~~~

- `snapcraft#6332 <https://github.com/canonical/snapcraft/issues/6322>`__ core26: YAML
  error messages refer to core24
- `snapcraft#6336 <https://github.com/canonical/snapcraft/issues/6336>`__ Snapcraft 9:
  snapcraftctl errors in core22 snaps


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

Learn about the new features, changes, and fixes in
:literalref:`@aleasto <https://github.com/aleasto>`,
:literalref:`@artivis <https://github.com/artivis>`,
:literalref:`@bepri <https://github.com/bepri>`,
:literalref:`@cmatsuoka <https://github.com/cmatsuoka>`,
:literalref:`@gcomneno <https://github.com/gcomneno>`,
:literalref:`@jahn-junior <https://github.com/jahn-junior>`,
:literalref:`@Kyuyrii <https://github.com/Kyuyrii>`,
:literalref:`@lengau <https://github.com/lengau>`,
:literalref:`@medubelko <https://github.com/medubelko>`,
:literalref:`@mr-cal <https://github.com/mr-cal>`,
:literalref:`@nicolasbock-canonical <https://github.com/nicolasbock-canonical>`,
:literalref:`@olivercalder <https://github.com/olivercalder>`,
:literalref:`@Rinconjr <https://github.com/Rinconjr>`,
:literalref:`@smethnani <https://github.com/smethnani>`,
:literalref:`@steinbro <https://github.com/steinbro>`,
:literalref:`@Tejas-Raj01 <https://github.com/Tejas-Raj01>`,
:literalref:`@tigarmo <https://github.com/tigarmo>`,
:literalref:`@tsteven4 <https://github.com/tsteven4>`,
:literalref:`@vismaytiwari <https://github.com/vismaytiwari>`,
:literalref:`@WaiHlyanMinThein17 <https://github.com/WaiHlyanMinThein17>`,
and :literalref:`@zhijie-yang <https://github.com/zhijie-yang>`
