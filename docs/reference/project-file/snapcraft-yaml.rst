.. meta::
    :description: Project configuration reference for Snapcraft. Find usage details and examples for every key in a snap's project file.

.. _reference-snapcraft-yaml:

snapcraft.yaml
==============

This reference describes the usage of and provides examples for every key in a snap's
project file, ``snapcraft.yaml``.

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

.. kitbash-field:: Project title

.. kitbash-field:: Project summary

.. kitbash-field:: Project description

.. kitbash-field:: Project version

.. kitbash-field:: Project base
    :override-description:
    :skip-examples:

    The base snap to be used as the run-time environment.

    If the ``build-base`` key is unset, then the ``base`` key also determines the build
    environment. For example, ``base: core24`` builds the snap in an Ubuntu 24.04
    environment.

    For more information about the ``base`` and ``build-base`` keys, see
    :ref:`reference-bases`.

    **Examples**

    .. code-block:: yaml

        base: core24

.. kitbash-field:: StableBaseProject build_base

.. kitbash-field:: Project source_code
    :override-type: str

.. kitbash-field:: Project license

.. kitbash-field:: Project contact
    :override-type: str | list[str]

.. kitbash-field:: Project issues
    :override-type: str | list[str]

.. kitbash-field:: Project donation
    :override-type: str | list[str]

.. kitbash-field:: Project website
    :override-type: str | list[str]

.. kitbash-field:: Project icon

.. kitbash-field:: Project adopt_info

.. kitbash-field:: Project environment

.. kitbash-field:: Project package_repositories
    :override-type: list[dict[str, Any]]

.. kitbash-field:: Project type
    :override-type: str

.. kitbash-field:: Project grade

.. kitbash-field:: Project confinement

.. kitbash-field:: Project compression

.. kitbash-field:: Project layout

.. kitbash-field:: Project passthrough

.. kitbash-field:: Project assumes

.. kitbash-field:: Project slots

.. kitbash-field:: Project lint

.. kitbash-field:: Project epoch

.. kitbash-field:: Project system_usernames

.. kitbash-field:: Project build_packages
    :override-type: list[str]

.. kitbash-field:: Project build_snaps
    :override-type: list[str]

.. kitbash-field:: Project ua_services

.. kitbash-field:: Project provenance


.. _reference-snapcraft-yaml-platform-keys:

Platform keys
-------------

.. kitbash-field:: Project platforms

.. kitbash-field:: Platform build_on
    :prepend-name: platforms.<platform-name>
    :override-type: str | list[str]

.. kitbash-field:: Platform build_for
    :prepend-name: platforms.<platform-name>
    :override-type: str | list[str]


.. _reference-snapcraft-yaml-architectures:

Architecture keys
-----------------

.. kitbash-field:: Project architectures

.. kitbash-field:: Architecture build_on
    :prepend-name: architectures.<architecture>
    :override-type: str | list[str]

.. kitbash-field:: Architecture build_for
    :prepend-name: architectures.<architecture>
    :override-type: str | list[str]


.. _reference-snapcraft-yaml-app-keys:

App keys
--------

.. kitbash-field:: Project apps

.. kitbash-field:: App command
    :prepend-name: apps.<app-name>

.. kitbash-field:: App autostart
    :prepend-name: apps.<app-name>

.. kitbash-field:: App common_id
    :prepend-name: apps.<app-name>

.. kitbash-field:: App bus_name
    :prepend-name: apps.<app-name>

.. kitbash-field:: App desktop
    :prepend-name: apps.<app-name>

.. kitbash-field:: App completer
    :prepend-name: apps.<app-name>

.. kitbash-field:: App stop_command
    :prepend-name: apps.<app-name>

.. kitbash-field:: App post_stop_command
    :prepend-name: apps.<app-name>

.. kitbash-field:: App start_timeout
    :prepend-name: apps.<app-name>

.. kitbash-field:: App stop_timeout
    :prepend-name: apps.<app-name>

.. kitbash-field:: App watchdog_timeout
    :prepend-name: apps.<app-name>

.. kitbash-field:: App reload_command
    :prepend-name: apps.<app-name>

.. kitbash-field:: App restart_delay
    :prepend-name: apps.<app-name>

.. kitbash-field:: App timer
    :prepend-name: apps.<app-name>

.. kitbash-field:: App daemon
    :prepend-name: apps.<app-name>

.. kitbash-field:: App after
    :prepend-name: apps.<app-name>

.. kitbash-field:: App before
    :prepend-name: apps.<app-name>

.. kitbash-field:: App refresh_mode
    :prepend-name: apps.<app-name>

.. kitbash-field:: App stop_mode
    :prepend-name: apps.<app-name>

.. kitbash-field:: App restart_condition
    :prepend-name: apps.<app-name>

.. kitbash-field:: App success_exit_status
    :prepend-name: apps.<app-name>

.. kitbash-field:: App install_mode
    :prepend-name: apps.<app-name>

.. kitbash-field:: App slots
    :prepend-name: apps.<app-name>

.. kitbash-field:: App plugs
    :prepend-name: apps.<app-name>

.. kitbash-field:: App aliases
    :prepend-name: apps.<app-name>

.. kitbash-field:: App environment
    :prepend-name: apps.<app-name>

.. kitbash-field:: App command_chain
    :prepend-name: apps.<app-name>

.. kitbash-field:: App sockets
    :prepend-name: apps.<app-name>

.. kitbash-field:: App daemon_scope
    :prepend-name: apps.<app-name>

.. kitbash-field:: App activates_on
    :prepend-name: apps.<app-name>

.. kitbash-field:: App passthrough
    :prepend-name: apps.<app-name>

.. kitbash-field:: App extensions
    :prepend-name: apps.<app-name>


.. _reference-snapcraft-yaml-part-keys:

Part keys
---------

.. kitbash-field:: Project parts
    :override-type: dict[str, Part]

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
    :override-description:

    Special identifiers that change certain features and behavior during the build.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``enable-usrmerge``
          - Fills the ``${CRAFT_PART_INSTALL}`` directory with a merged ``/usr``
            directory before running the part's build step.
        * - ``disable-usrmerge``
          - Prevents a merged ``/usr`` directory from being assembled for the build
            step. Available in lifecycles where the directory would be merged by
            default.

    The part's default behavior for executable patching is dependent on the base snap.
    The following options alter the behavior.

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
    dependencies <https://documentation.ubuntu.com/snapcraft/stable/explanation/parts-lifecycle/#processing-order-and-dependencies>`_.

.. kitbash-field:: PartSpec override_build
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec build_packages
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec build_snaps
    :prepend-name: parts.<part-name>

.. kitbash-field:: PartSpec organize_files
    :prepend-name: parts.<part-name>

    Files from the build environment can be organized into specific components. The
    destination path must start with ``(component/<component-name>)``, with the
    parentheses included. Source paths always reference the default build environment.

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

.. py:currentmodule:: snapcraft.models.project

.. kitbash-field:: App sockets
    :prepend-name: apps.<app-name>

.. kitbash-model:: Socket
    :prepend-name: sockets.<socket-name>
    :skip-description:


Hook keys
---------

.. kitbash-field:: Project hooks

.. kitbash-model:: Hook
    :prepend-name: hooks.<hook-type>
    :skip-description:


Component keys
--------------

.. kitbash-field:: Project components
    :override-type: dict[str, Component]

.. kitbash-model:: Component
    :prepend-name: components.<component-name>
    :skip-description:


Content plug keys
-----------------

.. kitbash-field:: Project plugs

.. kitbash-model:: ContentPlug
    :prepend-name: plugs.<plug-name>
    :skip-description:
