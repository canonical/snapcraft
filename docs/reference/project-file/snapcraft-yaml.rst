.. _reference-snapcraft-yaml:

snapcraft.yaml
==============

This reference describes the purpose, usage, and examples of all available keys in a
snap's project file, ``snapcraft.yaml``.

For keys that refer to file and directory locations, paths are always relative to the
snap's file system, not the host.


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

.. kitbash-field:: craft_application.models.project.Project package_repositories

.. kitbash-field:: project.Project hooks

.. kitbash-field:: project.Project components

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

.. kitbash-model:: craft_parts.parts.PartSpec
    :prepend-name: parts.<part-name>


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
