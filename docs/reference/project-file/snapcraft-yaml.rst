.. _reference-snapcraft-yaml:

snapcraft.yaml
==============

This reference describes the purpose, usage, and examples of all available keys in a
snap's project file, ``snapcraft.yaml``.

For keys that refer to file and directory locations, paths are always relative to the
snap's file system, not the host.


.. _reference-snapcraft-yaml-top-level-keys:

Top-level keys
--------------

The top-level keys dictate the snap's packaging information and the essential details of
how it builds.

Top-level details include a snap's name, version and description, alongside operational
values such as its confinement level and supported architectures.

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: Project name

.. py:currentmodule:: craft_application.models.project

.. kitbash-field:: Project title

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: Project version

.. py:currentmodule:: craft_application.models.project

.. kitbash-field:: Project license

.. kitbash-field:: Project summary

.. kitbash-field:: Project description

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: Project adopt_info

.. kitbash-field:: Project type

.. py:currentmodule:: craft_application.models.project

.. kitbash-field:: Project base

**Description**

The base snap to be used as the run-time environment.

If the ``build-base`` key is unset, then the ``base`` key also determines the build
environment. For example, ``base: core24`` builds the snap in an Ubuntu 24.04
environment.

For more information about the ``base`` and ``build-base`` keys, see
:ref:`reference-bases`.

**Examples**

.. code-block:: yaml

    base: core24

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: Project build_base

.. kitbash-field:: Project grade

.. kitbash-field:: Project confinement

.. kitbash-field:: Project source_code

.. kitbash-field:: Project contact

.. kitbash-field:: Project website

.. kitbash-field:: Project issues

.. kitbash-field:: Project donation

.. kitbash-field:: Project compression

.. kitbash-field:: Project icon

.. kitbash-field:: Project layout

.. kitbash-field:: Project passthrough

.. kitbash-field:: Project assumes

.. kitbash-field:: Project slots

.. kitbash-field:: Project lint

.. kitbash-field:: Project epoch

.. kitbash-field:: Project system_usernames

.. kitbash-field:: Project environment

.. kitbash-field:: Project build_packages

.. kitbash-field:: Project build_snaps

.. kitbash-field:: Project ua_services

.. kitbash-field:: Project provenance

.. kitbash-field:: Project platforms

.. kitbash-field:: Project architectures

.. kitbash-field:: Project apps

.. py:currentmodule:: craft_application.models.project

.. kitbash-field:: Project parts
    :override-type: dict[str, Part]

.. kitbash-field:: Project package_repositories
    :override-type: list[dict[str, Any]]

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: Project hooks

.. kitbash-field:: Project components
    :override-type: dict[str, Component]

.. kitbash-field:: Project plugs


.. _reference-snapcraft-yaml-app-keys:

App keys
--------

The ``apps`` key declares the programs and services that a snap operates on the host,
and details how they're executed and which resources they can access.

.. py:currentmodule:: snapcraft.models.app

.. kitbash-model:: App
    :prepend-name: apps.<app-name>


.. _reference-snapcraft-yaml-part-keys:

Part keys
---------

The ``parts`` key and its values declare the snap's :ref:`parts <explanation-parts>` and
detail how they're built.

.. py:currentmodule:: craft_parts.parts

.. Main keys

.. kitbash-field:: PartSpec plugin
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec after
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec disable_parallel
    :prepend-name: parts.<part-name>

.. Source keys

.. kitbash-field:: PartSpec source
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_type
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_checksum
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_branch
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_tag
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_commit
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_depth
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_submodules
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec source_subdir
    :prepend-name: parts.<part-name>

.. Pull step keys

.. kitbash-field:: PartSpec override_pull
    :prepend-name: parts.<part-name>

.. Build step keys

.. kitbash-field:: PartSpec build_environment
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec build_attributes
    :prepend-name: parts.<part-name>

**Description**

The part's default behavior for executable patching is dependent on the base snap.
The following options alter the behavior.

**Values**

.. list-table::
    :header-rows: 1

    * - Value
      - Description
    * - ``core22-step-dependencies``
      - For snaps using core20. Orders lifecycle steps as they are ordered for
        core22 and newer snaps.
    * - ``enable-patchelf``
      - For classically-confined snaps using core22 and newer. Patches executables
        for files primed by the part. By default, executables primed by the part
        aren't patched.
    * - ``no-patchelf``
      - For classically-confined snaps using core20. Disables executable patching
        for files primed by the part. If unset, patches all executables primed by
        the part.
    * - ``keep-execstack``
      - For snaps using core20. Retains the execstack for executables primed by the
        part.

For core20 snaps, the ``core22-step-dependencies`` customization alters the part
processing order to align with newer bases, where all parts are pulled prior to build.
For more details on part processing for core22 and newer, see `Processing order and
dependencies
<https://documentation.ubuntu.com/snapcraft/stable/explanation/parts-lifecycle/#processing-order-and-dependencies>`_.

.. kitbash-field:: PartSpec override_build
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec build_packages
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec build_snaps
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec organize_files
    :prepend-name: parts.<part-name>

Files from the build environment can be organized into specific components. The
destination path must start with ``(component/<component-name>)``, with the parentheses
included. Source paths always reference the default build environment.


.. Stage step keys

.. kitbash-field:: PartSpec stage_files
    :prepend-name: parts.<part-name>
    :override-type: list[str]

.. kitbash-field:: PartSpec stage_packages
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec stage_snaps
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec override_stage
    :prepend-name: parts.<part-name>

.. Prime step keys

.. kitbash-field:: PartSpec prime_files
    :prepend-name: parts.<part-name>
    :override-type: list[str]

.. kitbash-field:: PartSpec override_prime
    :prepend-name: parts.<part-name>

.. Permission keys

.. kitbash-field:: PartSpec permissions
    :prepend-name: parts.<part-name>

.. py:currentmodule:: craft_parts.permissions

.. kitbash-field:: Permissions path
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: Permissions owner
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: Permissions group
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: Permissions mode
    :prepend-name: parts.<part-name>.permissions.<permission>


Socket keys
-----------

.. py:currentmodule:: snapcraft.models.app

.. kitbash-model:: Socket
    :prepend-name: sockets.<socket-name>


Hook keys
---------

.. py:currentmodule:: snapcraft.models.project

.. kitbash-model:: Hook
    :prepend-name: hooks.<hook-type>


Component keys
--------------

.. kitbash-model:: Component
    :prepend-name: components.<component-name>


Content plug keys
-----------------

.. kitbash-model:: ContentPlug
    :prepend-name: plugs.<plug-name>
