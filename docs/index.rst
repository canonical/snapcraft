.. meta::
    :description: Documentation for Snapcraft, the tool for packaging software into the snap container format.


Snapcraft
=========

**Snapcraft** is the tool for packaging software into the snap container format.

It builds and bundles Linux software of all kinds and sources, and is operated through a
command-line interface.

Snapcraft is compatible with many languages and frameworks, including Python, Rust, Go,
and GNOME. It provides debugging and testing capabilities to ready a snap for
publication to the Snap Store or a private store.

Snapcraft is for developers, package maintainers, fleet administrators, and hobbyists
who publish software for desktop and IoT devices.


In this documentation
---------------------


Get started
~~~~~~~~~~~

Package your first snap and gain familiarity with crafting basics.

.. domain::

  .. slice:: Tutorial

    :doc:`tutorials/craft-a-snap`

  .. slice:: Snap project file

    :doc:`explanation/snapcraft-yaml`
    :doc:`reference/snapcraft-yaml`

  .. slice:: Installation

    :doc:`how-to/set-up-snapcraft`
    :doc:`reference/system-requirements`

  .. slice:: CLI

    :doc:`reference/commands`


Language and framework support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft packages software built in one or more programming languages and frameworks. Use these guides to package your app for production.

.. domain::

  .. slice:: Languages

    :doc:`how-to/integrations/craft-a-python-app`
    :doc:`how-to/integrations/craft-a-c-or-cpp-app`
    :doc:`how-to/integrations/craft-a-java-app`
    :doc:`how-to/integrations/craft-a-go-app`
    :doc:`how-to/integrations/craft-a-rust-app`
    :doc:`how-to/integrations/craft-a-node-app`
    :doc:`how-to/integrations/craft-an-ros-2-app`
    :doc:`how-to/integrations/craft-an-ros-1-app`

  .. slice:: Frameworks

    :doc:`how-to/integrations/craft-a-gtk4-app`
    :doc:`how-to/integrations/craft-a-gtk3-app`
    :doc:`how-to/integrations/craft-a-gtk2-app`
    :doc:`how-to/integrations/craft-a-qt5-kde-app`
    :doc:`how-to/integrations/craft-a-flutter-app`
    :doc:`how-to/integrations/craft-an-electron-app`
    :doc:`how-to/integrations/craft-a-dotnet-app`
    :doc:`how-to/integrations/craft-a-moos-app`

  .. slice:: Macros

    :doc:`how-to/extensions/use-an-extension`
    :doc:`how-to/extensions/expand-extensions`
    :doc:`how-to/extensions/list-extensions`
    :doc:`how-to/extensions/enable-experimental-extensions`
    :doc:`how-to/extensions/use-the-env-injector-extension`


Snap definition
~~~~~~~~~~~~~~~

Define the software sources, dependencies, assets, and runtime of your snap.

.. domain::

  .. slice:: Package information

    :doc:`how-to/crafting/configure-package-information`
    :doc:`reference/external-package-information`

  .. slice:: Platforms

    :doc:`how-to/crafting/select-platforms`
    :doc:`how-to/integrations/craft-a-cross-compiled-app`
    :doc:`explanation/platforms`
    :doc:`reference/platforms`
    :doc:`reference/advanced-grammar`

  .. slice:: Software sources

    :doc:`how-to/crafting/override-the-parts-lifecycle`
    :doc:`reference/part-environment-variables`
    :doc:`reference/plugins`
    :doc:`explanation/parts`
    :doc:`explanation/parts-lifecycle`
    :doc:`reference/parts-steps`
    :doc:`explanation/build-overrides`

  .. slice:: Software dependencies

    :doc:`how-to/crafting/specify-a-base`
    :doc:`reference/bases`
    :doc:`how-to/crafting/manage-dependencies`
    :doc:`reference/package-repositories`

  .. slice:: File organization

    :doc:`how-to/integrations/craft-a-pre-built-app`
    :doc:`how-to/crafting/include-local-files-and-remote-resources`
    :doc:`reference/layouts`
    :doc:`how-to/crafting/use-layouts`
    :doc:`explanation/components`
    :doc:`reference/components`
    :doc:`how-to/crafting/create-a-component`
    :doc:`/common/craft-parts/explanation/filesets`

  .. slice:: Runtime

    :doc:`explanation/snap-configurations`
    :doc:`how-to/crafting/add-a-snap-configuration`
    :doc:`reference/snapshots`
    :doc:`reference/hooks`


Snap builds
~~~~~~~~~~~

Build snaps with a local provider or remotely with Launchpad.

.. domain::

  .. slice:: Build configuration

    :doc:`how-to/select-a-build-provider`
    :doc:`explanation/snap-build-process`
    :doc:`reference/build-environment-options`

  .. slice:: Distributed building

    :doc:`how-to/crafting/build-snap-remotely`
    :doc:`explanation/remote-build`
    :doc:`how-to/crafting/reuse-packages-between-builds`


Snap stores
~~~~~~~~~~~

Publish snaps to the Canonical Snap Store or a vendored store. Release updates to snaps and monitor their status and usage.

.. domain::

  .. slice:: Stores

    :doc:`how-to/publishing/authenticate`
    :doc:`how-to/publishing/register-a-snap`

  .. slice:: Publishing

    :doc:`how-to/publishing/publish-a-snap`
    :doc:`how-to/publishing/manage-revisions-and-releases`
    :doc:`explanation/snap-publishing-process`
    :doc:`reference/channels`

  .. slice:: Monitoring

    :doc:`how-to/publishing/check-a-snaps-status`
    :doc:`how-to/publishing/get-snap-metrics`
    :doc:`reference/metrics`


Security and correctness
~~~~~~~~~~~~~~~~~~~~~~~~

Provide access to only required system resources, and use the linting and debugging features to refine your snap.

.. domain::

  .. slice:: Sandboxing

    :doc:`how-to/crafting/enable-classic-confinement`
    :doc:`explanation/classic-confinement`
    :doc:`explanation/interfaces`

  .. slice:: Validation

    :doc:`how-to/debugging/debug-a-snap`
    :doc:`how-to/debugging/debug-with-gdb`
    :doc:`how-to/debugging/use-the-classic-linter`
    :doc:`how-to/debugging/use-the-library-linter`
    :doc:`how-to/debugging/use-the-metadata-linter`
    :doc:`how-to/debugging/use-the-gpu-linter`
    :doc:`how-to/debugging/disable-a-linter`
    :doc:`reference/linters`

  .. slice:: Security

    :doc:`explanation/cryptography`


Snap support
~~~~~~~~~~~~

Plan your deployment with Snapcraft and keep your snap secure and up-to-date.

.. domain::

  .. slice:: Deployment

    :doc:`reference/support-schedule`

  .. slice:: Upgrading

    :doc:`how-to/crafting/manage-data-compatibility`
    :doc:`how-to/change-bases/change-from-core18-to-core20`
    :doc:`how-to/change-bases/change-from-core20-to-core22`
    :doc:`how-to/change-bases/change-from-core22-to-core24`
    :doc:`how-to/change-bases/change-from-core24-to-core26`


How this documentation is organized
-----------------------------------

The Snapcraft documentation embodies the `Diátaxis framework <https://diataxis.fr/>`__.

* The :ref:`tutorial <tutorials>` is a lesson that steps through the main process of
  packaging a snap.
* :ref:`how-to-guides` contain directions for crafting and debugging snaps.
* :ref:`References <reference>` describe the structure and function of the individual components in
  Snapcraft.
* :ref:`Explanations <explanation>` aid in understanding the concepts and relationships
  of Snapcraft as a system.


Project and community
---------------------

Snapcraft is a member of the Canonical family. It's an open source project that warmly
welcomes community projects, contributions, suggestions, fixes and constructive
feedback.


Get involved
~~~~~~~~~~~~

- `Snapcraft Matrix channel <https://matrix.to/#/#snapcraft:ubuntu.com>`__
- `Snapcraft forum <https://forum.snapcraft.io/>`__
- `Contribute to Snapcraft development
  <https://github.com/canonical/snapcraft/blob/main/CONTRIBUTING.md>`__
- :ref:`contribute-to-this-documentation`


Releases and support
~~~~~~~~~~~~~~~~~~~~

- :ref:`release-notes`
- :ref:`reference-support-schedule`


Governance and policies
~~~~~~~~~~~~~~~~~~~~~~~

- `Ubuntu Code of Conduct <https://ubuntu.com/community/docs/ethos/code-of-conduct>`__
- `Canonical Contributor License Agreement
  <https://ubuntu.com/legal/contributors>`__


.. toctree::
    :hidden:

    tutorials/index
    how-to/index
    reference/index
    explanation/index

.. toctree::
    :hidden:

    release-notes/index
    contribute/index
    about-documentation
