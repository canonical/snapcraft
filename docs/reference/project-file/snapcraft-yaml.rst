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

.. kitbash-field:: project.Project name

.. kitbash-field:: craft_application.models.project.Project title

.. kitbash-field:: project.Project version

.. kitbash-field:: craft_application.models.project.Project license

.. kitbash-field:: craft_application.models.project.Project summary

.. kitbash-field:: craft_application.models.project.Project description

.. kitbash-field:: project.Project adopt_info

.. kitbash-field:: project.Project type

.. kitbash-field:: craft_application.models.project.Project base

.. kitbash-field:: project.Project build_base

.. kitbash-field:: project.Project grade

.. kitbash-field:: project.Project confinement

.. kitbash-field:: project.Project source_code

.. kitbash-field:: project.Project contact

.. kitbash-field:: project.Project website

.. kitbash-field:: project.Project issues

.. kitbash-field:: project.Project donation

.. kitbash-field:: project.Project compression

.. kitbash-field:: project.Project icon

.. kitbash-field:: project.Project layout

.. kitbash-field:: project.Project passthrough

.. kitbash-field:: project.Project assumes

.. kitbash-field:: project.Project slots

.. kitbash-field:: project.Project lint

.. kitbash-field:: project.Project epoch

.. kitbash-field:: project.Project system_usernames

.. kitbash-field:: project.Project environment

.. kitbash-field:: project.Project build_packages

.. kitbash-field:: project.Project build_snaps

.. kitbash-field:: project.Project ua_services

.. kitbash-field:: project.Project provenance

.. kitbash-field:: project.Project platforms

.. kitbash-field:: project.Project architectures

.. kitbash-field:: project.Project apps

.. kitbash-field:: craft_application.models.project.Project parts
    :override-type: dict[str, Part]

.. kitbash-field:: craft_application.models.project.Project package_repositories
    :override-type: list[dict[str, Any]]

.. kitbash-field:: project.Project hooks

.. kitbash-field:: project.Project components
    :override-type: dict[str, Component]

.. kitbash-field:: project.Project plugs


.. _reference-snapcraft-yaml-app-keys:

App keys
--------

The ``apps`` key declares the programs and services that a snap operates on the host,
and details how they're executed and which resources they can access.

.. kitbash-model:: project.App
    :prepend-name: apps.<app-name>


.. _reference-snapcraft-yaml-part-keys:

Part keys
---------

The ``parts`` key and its values declare the snap's :ref:`parts <explanation-parts>` and
detail how they're built.

.. Main keys

.. kitbash-field:: craft_parts.parts.PartSpec plugin
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec after
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec disable_parallel
    :prepend-name: parts.<part-name>

.. Source keys

.. kitbash-field:: craft_parts.parts.PartSpec source
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_type
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_checksum
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_branch
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_tag
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_commit
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_depth
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_submodules
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec source_subdir
    :prepend-name: parts.<part-name>

.. Pull step keys

.. kitbash-field:: craft_parts.parts.PartSpec override_pull
    :prepend-name: parts.<part-name>

.. Build step keys

.. kitbash-field:: craft_parts.parts.PartSpec build_environment
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec build_attributes
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

.. kitbash-field:: craft_parts.parts.PartSpec override_build
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec build_packages
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec build_snaps
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec organize_files
    :prepend-name: parts.<part-name>

Files from the build environment can be organized into specific components. The
destination path must start with ``(component/<component-name>)``, with the parentheses
included. Source paths always reference the default build environment.


.. Stage step keys

.. kitbash-field:: craft_parts.parts.PartSpec stage_files
    :prepend-name: parts.<part-name>
    :override-type: list[str]

.. kitbash-field:: craft_parts.parts.PartSpec stage_packages
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec stage_snaps
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.parts.PartSpec override_stage
    :prepend-name: parts.<part-name>

.. Prime step keys

.. kitbash-field:: craft_parts.parts.PartSpec prime_files
    :prepend-name: parts.<part-name>
    :override-type: list[str]

.. kitbash-field:: craft_parts.parts.PartSpec override_prime
    :prepend-name: parts.<part-name>

.. Permission keys

.. kitbash-field:: craft_parts.parts.PartSpec permissions
    :prepend-name: parts.<part-name>

.. kitbash-field:: craft_parts.permissions.Permissions path
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: craft_parts.permissions.Permissions owner
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: craft_parts.permissions.Permissions group
    :prepend-name: parts.<part-name>.permissions.<permission>

.. kitbash-field:: craft_parts.permissions.Permissions mode
    :prepend-name: parts.<part-name>.permissions.<permission>


Socket keys
-----------

.. kitbash-model:: project.Socket
    :prepend-name: sockets.<socket-name>


Hook keys
---------

.. kitbash-model:: project.Hook
    :prepend-name: hooks.<hook-type>


Component keys
--------------

.. kitbash-model:: project.Component
    :prepend-name: components.<component-name>


Content plug keys
-----------------

.. kitbash-model:: project.ContentPlug
    :prepend-name: plugs.<plug-name>
