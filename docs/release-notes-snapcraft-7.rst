.. 30464.md

.. _release-notes-snapcraft-7:

Release notes: Snapcraft 7
==========================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 7.0 <https://github.com/snapcore/snapcraft/releases/tag/7.0>`__, a major update to the tool used to build snap packages.

This release marks the end of an initial process to rebuild our tools to work across a variety of projects, and many of the changes reflect that development work. This includes a new lifecycle and the introduction of `Craft Parts <https://craft-parts.readthedocs.io/en/latest/>`__ as a generic cross-project mechanism for obtaining data from different sources.

Among the many any other updates, fixes and additions, the following are what we consider its highlights:

-  Support for the new ``core22`` base
-  LXD promoted to be the new default build environment
-  A new store authentication mechanism

For general details, including installation instructions, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__, or take a look at `Snapcraft release notes <https://snapcraft.io/docs/snapcraft-release-notes>`__ for other *Snapcraft* releases.

core22 support
--------------

These changes are now applicable when switching to ``core22``:

-  A new ``craftctl`` tool replaces ``snapcraftctl``\  This change also requires that:

   -  ``craftctl default`` replaces ``snapcraftctl <step-name>``
   -  ``craftctl set version=<value>`` replaces ``snapcraftctl set-version <value>``

-  ``CRAFT_*`` environment variables are defined during step execution, replacing ``SNAPCRAFT_*`` variables.
-  ``try`` has been removed from :ref:`advanced grammar <snapcraft-advanced-grammar>`. This enables a more predictable planning phase without requiring an execution environment to attest what is tried. Grammar has now been generalised enough to allow for it to be added seamlessly to all snapcraft supported keywords.
-  An error is now generated when duplicate keys are used in snapcraft.yaml.
-  Snapcraft now makes use of the global environment keyword instead of command-chain (which allows for easy overriding)

The following are not currently supported with ``core22`` :

- Automatic classic snap building support (ORIGIN paths and linker loader setup)
- Plugins: crystal, qmake
- Some source handlers: 7zip, mercurial, bazaar, deb, rpm
- Support for user-defined plugins.

Missing features will be ported and added to upcoming 7.x releases. If your snap relies on any of these features, please wait for the next releases to port them to use ``base: core22``.

See :ref:`How to migrate from core20 to core22 10 <micro-howto-migrate-from-core20-to-core22>` for help on migrating your snaps to the new base.

Build Providers
---------------

The build environment is now created through a new API called `Craft Providers <https://craft-providers.readthedocs.io/en/latest/>`__, and consequently, Snapcraft now defaults to using :ref:`LXD <build-providers>`.

Command Line Interface
----------------------

Packing or Snapping
~~~~~~~~~~~~~~~~~~~

To align with the other *craft* tools, the snapping process now uses the pack command rather than the now deprecated *snap* command:

::

   snapcraft pack

Running :command:`snapcraft` with no arguments is still supported and defaults to packing.

Store operations with externally generated credentials
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft has migrated to using a new `Craft Store <https://craft-store.readthedocs.io/en/latest/>`__ tool to access the snap store. As a result, The authentication mechanism has changed, impacting any project already using CI/CD.

The ``snapcraft login --with`` command structure is no longer supported or required. The value of export login now needs to be exported into the appropriate environment variable (i.e.; ``SNAPCRAFT_STORE_CREDENTIALS``).

Additionally, a working keyring is required to be able to store credentials locally.

Experimental login
~~~~~~~~~~~~~~~~~~

To change the authentication mechanism, replace ``snapcraft login --enable-experimental-login``, with ``SNAPCRAFT_STORE_AUTH=candid``.

Listing snap names
~~~~~~~~~~~~~~~~~~

The ``snapcraft list`` command to show which snap names are associated with your account has been deprecated and replaced with ``snapcraft names``.

::

   snapcraft names
