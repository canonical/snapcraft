.. _release-8.8:

Snapcraft 8.8 release notes
===========================

14 April 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.8.


Requirements and compatibility
------------------------------

See :ref:`System requirements<system-requirements>` for information on the minimum
hardware and installed software.


What's new
----------

Snapcraft 8.8 brings the following features, integrations, and improvements.

Cross-compiling for core24
~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, cross-compiling for ``core24`` snaps only worked for compatible
architectures: building on AMD64 for I386 and building on ARM64 for ARMHF.

This restriction has been dropped, so ``package-repositories`` and ``build-packages``
keys can now define entries for any :ref:`supported architecture
<supported-architectures>`.

For more information, see the :ref:`example-cross-compiled-app` how-to.

Remote build improvements
~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`remote-build command <ref_commands_remote-build>` can now request builds on
Launchpad that build on one architecture and build for a different architecture.

Additionally, architecture-independent snaps have improved handling. Previously, a snap
with a ``build-for: [all]`` entry would always build on AMD64. Now, it builds on one of
the ``build-on`` architectures listed in the project file.


KDE Neon
~~~~~~~~

A new KDE extension, ``kde-neon-qt6``, has been added for for standalone Qt 6
applications using the KDE neon 6 stack.

Additionally, many improvements and fixes have been made to the build environments for
KDE neon extensions:

- The ``mesa`` directories have been added to the ``LD_LIBRARY_PATH``.
- Duplicate paths to ``X11`` directories have been removed.
- The seperator for ``CMAKE_PREFIX_PATH`` has been fixed.
- ``CMAKE_FIND_ROOT_PATH`` has been removed.

JLink plugin
~~~~~~~~~~~~

A new plugin has been added that uses the `jlink`_ tool to create smaller, optimized
Java runtimes specific for your snap's JARs. See the :ref:`craft_parts_jlink_plugin`
reference for more information.


Minor features
--------------

Snapcraft 8.8 brings the following minor changes.

Security file
~~~~~~~~~~~~~

A `security policy`_ has been added to the project, detailing Snapcraft's release cycle
and how to report vulnerabilities.

Support for short hashes
~~~~~~~~~~~~~~~~~~~~~~~~

Previously, the ``source-commit`` key for parts could only accept full length (40
character) hashes. Now, ``source-commit`` can accept short hashes.


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@bepri<https://github.com/bepri>`,
:literalref:`@dariuszd21<https://github.com/dariuszd21>`,
:literalref:`@lengau<https://github.com/lengau>`,
:literalref:`@medubelko<https://github.com/medubelko>`,
:literalref:`@mr-cal<https://github.com/mr-cal>`,
:literalref:`@ScarlettGatelyMoore<https://github.com/ScarlettGatelyMoore>`,
and :literalref:`@sergiusens<https://github.com/sergiusens>`

.. _jlink: https://docs.oracle.com/en/java/javase/21/docs/specs/man/jlink.html
.. _security policy: https://github.com/canonical/snapcraft/blob/main/SECURITY.md
