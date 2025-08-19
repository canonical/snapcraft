.. _release-8.11:

Snapcraft 8.11 release notes
============================

6 August 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.11.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.11 brings the following features, integrations, and improvements.

Unified behavior for the pack command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a new base is introduced in Snapcraft, it expects new vocabulary and expressions in
the project file. Projects on old bases aren't affected, and can continue building using
the older vocabulary. It's only when a crafter switches their existing project to a new
base that the breaking changes are exposed, which leads to an in-depth migration.

This strategy has downsides. In particular, the user experience can be inconsistent when
building snaps with different bases. Running ``snapcraft pack`` would behave differently
depending on the base.

With this release, the pack command for core20 snaps is ported to have the same
functionality as the pack command for core22 and core24 snaps. With no arguments,
running ``snapcraft`` builds and packages the snap. If a directory is passed as the
first positional argument, Snapcraft skips the build process and instead packs the
directory as a snap.

Unified verbosity arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are five levels of verbosity to choose from when building core22 and core24 snaps:
quiet, brief, verbose, debug, and trace. These can be selected with the command line
arguments ``--quiet``, ``--verbose``,  and ``--verbosity <level>`` or the environment
variable ``CRAFT_VERBOSITY_LEVEL``.

However, these options weren't supported by all commands or when building core20 snaps.
This led to an inconsistent user experience. To remedy this, we've made these verbosity
options available for all commands when building snaps with core20, core22, and
core24 bases.


Minor features
--------------

Contribution guidelines
~~~~~~~~~~~~~~~~~~~~~~~

In an effort to improve the contributor experience, we've updated our `contribution
guidelines <https://github.com/canonical/snapcraft/blob/main/CONTRIBUTING.md>`_ and
added a new :ref:`contribute-to-this-documentation` page, which explains the Snapcraft
documentation's design principles and how to get involved.

Extra modules parameter for the jlink plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`craft_parts_jlink_plugin` now has a ``jlink-extra-modules`` parameter
to add additional modules to the OpenJDK image.


Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Snap file name changes
~~~~~~~~~~~~~~~~~~~~~~

The file names for core24 snaps are now derived from the name of the entry in the
``platforms`` key instead of the value of its ``build-for`` key.

For example, consider the following snippet:

.. code-block:: yaml
  :caption: snapcraft.yaml

  name: hello-world
  base: core24
  version: 1.0
  platforms:
    rpi:
      build-on: [arm64]
      build-for: [arm64]
    rpi-debug:
      build-on: [arm64]
      build-for: [arm64]

Previously, Snapcraft would build both snaps and use the same name,
``hello-world_1.0_arm64.snap``, for both snaps. This would result in the first snap
being overwritten by the second snap.

Now, Snapcraft builds two separate snaps, ``hello-world_1.0_rpi.snap`` and
``hello-world_1.0_rpi-debug.snap``.

Removed core20 error reporting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When an internal error occurred while building a core20 snap, Snapcraft would prompt
users to submit an anonymous report to Sentry.

This has been removed in favor of submitting bug reports to Snapcraft's `GitHub issues
<https://github.com/canonical/snapcraft/issues>`_ page.

As a result, the ``SNAPCRAFT_ENABLE_ERROR_REPORTING`` and
``SNAPCRAFT_ENABLE_SILENT_REPORT`` environment variables are no longer in use.


Scheduled feature deprecations
------------------------------

The following features will be deprecated in a future major release:

Running Snapcraft without a command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If Snapcraft is run without a command (``snapcraft``), the default behavior is to
pack a snap.

This release adds a deprecation warning when Snapcraft is run without a command.
``snapcraft pack`` should be used instead.


Fixed bugs and issues
---------------------

The following issues have been resolved in Snapcraft 8.11.

.. _release-notes-fixes-8.11.0:

Snapcraft 8.11.0
~~~~~~~~~~~~~~~~

- Multi-line details in error messages now start on a new line.
- If the project file is empty, Snapcraft now emits an error instead of a traceback.

.. _release-notes-fixes-8.11.1:

Snapcraft 8.11.1
~~~~~~~~~~~~~~~~

- `#5298`_ Verbosity arguments are now parsed for all non-lifecycle commands.

.. _release-notes-fixes-8.11.2:

Snapcraft 8.11.2
~~~~~~~~~~~~~~~~

- `#5635`_ If installing a GPG key from a keyserver for a package repository fails,
  Snapcraft now retries installing the key using the proxy.

Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@ahkazak23<https://github.com/ahkazak23>`,
:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@steinbro<https://github.com/steinbro>`,
:literalref:`@jahn-junior<https://github.com/jahn-junior>`,
:literalref:`@lengau<https://github.com/lengau>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`, and
:literalref:`@upils<https://github.com/upils>`.

.. _#5298: https://github.com/canonical/snapcraft/issues/5298
.. _#5635: https://github.com/canonical/snapcraft/issues/5635
