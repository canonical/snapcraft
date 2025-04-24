.. _reference-snapcraft-yaml:

snapcraft.yaml
==============

This page provides brief descriptions and examples for each key that can be included
in a snap's project file.


Top-level keys
--------------

The top-level keys and values in snapcraft.yaml provide the snap build process, and the
store, with the overarching details of a snap.

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


App keys
--------

The ``apps`` key and its values in a snap's project file detail the applications and
services that a snap wants to expose, including how they're executed and which resources
they can access.

.. kitbash-model:: project.App
    :prepend-name: apps.<app-name>


Part keys
---------

The main building blocks of a snap are called parts. They are used to declare pieces of
code that will be pulled into your snap package. The ``parts`` keys and its values in
a snap's project file detail how parts are configured and built.

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
