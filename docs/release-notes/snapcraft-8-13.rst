.. _release-8.13:

Snapcraft 8.13 release notes
============================

21 October 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.13.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.13 brings the following features, integrations, and improvements.

Experimental core26 support
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Experimental support for building snaps for the core26 base is now available. While
it's in testing, snaps built with core26 might break, even from patch releases of
Snapcraft. To build with core26, set the following in your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core26
    build-base: devel


Set component versions dynamically
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, a component could only set its version statically, when first declared.

Parts can now set a component versions dynamically. If a component points to a part with
the ``adopt-info`` key, the part can call craftctl to set the version.

For detailed guidance, see
:ref:`how-to-customize-the-build-and-part-variables-access-project-variables`.


Documentation submodule name change
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Git submodule containing documentation components has been renamed to
``sphinx-docs-starter-pack`` to match its parent repository.

If you're a returning contributor to the project, after you pull the latest commits, run
the following commands in your local repository to sync the submodule change:

.. code-block::

    git submodule sync
    git submodule update --init --recursive
    git clean -ffd


New .NET extensions
~~~~~~~~~~~~~~~~~~~

The experimental dotnet8, dotnet9, and dotnet10 :ref:`reference-dotnet-extensions` have
been added for core24 snaps.


Minor features
--------------

Renamed commands
~~~~~~~~~~~~~~~~

In accordance with our latest design standards, all commands starting with
``list-`` have been renamed. The word "list" will no longer be used. The old
command names will continue working until Snapcraft 9.

The affected commands are:

.. list-table::
    :header-rows: 1

    * - Old command
      - New command
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

Additions to the GNOME extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`reference-gnome-extension` now defines a the ``SNAPCRAFT_GNOME_SDK``
environment variable, which points to the path of the GNOME SDK used by the snap.

Documentation changes
~~~~~~~~~~~~~~~~~~~~~

A :ref:`new how-to <how-to-reuse-packages-between-builds>` for setting up a local
package cache for Snapcraft has been added.

Lifecycle commands
~~~~~~~~~~~~~~~~~~

When you run a lifecycle command with ``--debug``, Snapcraft will now print the error
before opening a shell into the build environment.

``stage-packages``
~~~~~~~~~~~~~~~~~~

Fixed an issue that caused Apt to warn about directory ownership when including
``stage-packages``.

Autotools plugin
~~~~~~~~~~~~~~~~

The :ref:`craft_parts_autotools_plugin` now supports the :ref:`disable-parallel
<PartSpec.disable_parallel>` key to force builds using the plugin to run using a single
job.

Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.13.

Snapcraft 8.13.0
~~~~~~~~~~~~~~~~

- `#5618`_ The ``sign-build`` command now correctly accepts a snap file to sign.

- `#5789`_ Using the architecture shorthand for multiple architectures on ``core20``
  snaps emits a warning about the potentially unexpected behavior.

- `#5794`_ The ``source-code`` metadata field can now be ignored during linting.

- `#5704`_ Linter results will now always report the highest severity detected, rather
  than the most recently seen severity.

Snapcraft 8.13.1
~~~~~~~~~~~~~~~~

- `#5843`_ The changes to the project model parsing from Snapcraft 8.13.0 have been
  reverted to give more time for testing.

Snapcraft 8.13.2
~~~~~~~~~~~~~~~~

- `craft-providers#850`_ Fix the "Couldn't get EOL data" errors when packing snaps with
  the ``devel`` build base.


Contributors
------------

We would like to express a big thank you to all the people who contributed to this
release.

:literalref:`@3v1n0<https://github.com/3v1n0>`
:literalref:`@Amanlem<https://github.com/Amanlem>`
:literalref:`@bepri<https://github.com/bepri>`
:literalref:`@fabolhak<https://github.com/fabolhak>`
:literalref:`@jahn-junior<https://github.com/jahn-junior>`
:literalref:`@lengau<https://github.com/lengau>`
:literalref:`@mateusrodrigues<https://github.com/mateusrodrigues>`
:literalref:`@medubelko<https://github.com/medubelko>`
:literalref:`@mr-cal<https://github.com/mr-cal>`
:literalref:`@Nalin-Kumar-Gupta<https://github.com/Nalin-Kumar-Gupta>`
:literalref:`@sergio-costas<https://github.com/sergio-costas>`
:literalref:`@steinbro<https://github.com/steinbro>`
:literalref:`@tigarmo<https://github.com/tigarmo>`


.. _#5618: https://github.com/canonical/snapcraft/issues/5618
.. _#5789: https://github.com/canonical/snapcraft/issues/5789
.. _#5794: https://github.com/canonical/snapcraft/issues/5794
.. _#5704: https://github.com/canonical/snapcraft/issues/5704
.. _#5843: https://github.com/canonical/snapcraft/issues/5843
.. _craft-providers#850: https://github.com/canonical/craft-providers/pull/850
