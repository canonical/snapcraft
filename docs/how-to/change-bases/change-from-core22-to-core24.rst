.. _how-to-change-from-core22-to-core24:

Change from core22 to core24
============================

This guide describes the process for migrating a snap that uses core22 as its base to
core24.

Support for core24 is provided by the `Craft Application
<https://github.com/canonical/craft-application>`_ library. Craft Application introduced
changes to the lifecycle, added a new remote builder, and replaced the concept of
``architectures`` with ``platforms``. These updates inform the migration process.

Deprecated features
-------------------

The following features are deprecated in core22 and removed in
core24:

-  The parameters ``--enable-manifest`` and
   ``--manifest-image-information``. Generation of the build’s manifest
   in core24 is controlled exclusively by the ``SNAPCRAFT_BUILD_INFO``
   environment variable, and the image information parameter is
   completely removed.
-  The ``snapcraftctl`` command in scriptlets is no longer supported and
   should be replaced with ``craftctl``.
-  The ``architectures`` key has been replaced by
   ``platforms``. See details below.
-  The environment variable ``CRAFT_ARCH_TRIPLET``. Use
   ``CRAFT_ARCH_TRIPLET_BUILD_FOR`` instead.
-  The environment variable ``CRAFT_TARGET_ARCH``. Use
   ``CRAFT_ARCH_BUILD_FOR`` instead.

Destructive builds
------------------

-  :ref:`Destructive builds <reference-build-environment-options-destructive>`
   that generate more than one snap are no longer supported, as this feature was
   never intended.
-  Destructive builds that generate no snaps are no longer supported, as
   this feature was never intended. It used to be a no-op but now will
   fail with an error message.
-  Destructive builds with a project whose build-base does not match the
   host environment will fail with an error message. An exception is
   made for devel build-bases. In this case the only requirement is
   that the host be an Ubuntu system.

Remote build
------------

A new remote builder has been introduced. While the previous remote builder is
available for older bases, core24 snaps must use the new remote builder.

-  The :ref:`remote build command <explanation-remote-build>` now only allows
   a single active remote build per project for core24 snaps. As a consequence,
   ``snapcraft remote-build --recover`` no longer accepts a
   ``--build-id`` parameter. Instead, snapcraft will recover the remote
   build for the project in the current working directory.
-  Projects must be at the top level of a git repository.
-  Projects cannot be shallow clones (``git clone --depth 1``).

Platforms
~~~~~~~~~

A new ``platforms`` key is the way for core24 snaps to declare the systems where the
snap should be built. This aligns with :external+charmcraft:ref:`Charmcraft's
<charmcraft-yaml-key-bases>` and :external+rockcraft:ref:`Rockcraft's
<rockcraft-yaml-platform-keys>` ``platforms`` key.

``platforms`` is an optional key. If it's not defined, then Snapcraft will
build a snap for the host architecture. Valid architectures are defined
:ref:`here <supported-architectures>`.

The syntax is:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      <platform 1>:
        build-on: [<arch 1>, <arch 2>]
        build-for: [<arch 1>]
      <platform 2>:
        build-on: [<arch 3>]
        build-for: [<arch 4>]
      ...

The definitions of ``build-on`` and ``build-for`` are unchanged (see
core22 definitions :ref:`here <reference-architectures-core22>`).

Similar to core22, only one ``build-for`` architecture is supported and
it may be defined as a list or a string.

The platform name is a free-form string. Some simplifications are gained
if the platform name is a supported architecture:

-  ``build-on`` and ``build-for`` may be omitted
-  ``build-for`` may be omitted if ``build-on`` is defined

When migrating an existing snap to core24, existing ``architectures``
definitions can be rewritten as ``platforms``. For example:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [amd64]
      - build-on: [amd64, arm64]
        build-for: [arm64]

can be migrated to core24 as:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
      arm64:
        build-on: [amd64, arm64]
        build-for: [arm64]

For more information on platforms see the :ref:`how-to <how-to-select-architectures>`,
:ref:`explanation <explanation-architectures>`, and
:ref:`reference <reference-architectures>` pages.

Known issues
------------

-  The command ``snapcraft try`` is not yet supported for core24 snaps.
