.. _reference-snapcraft-yaml:

snapcraft.yaml
==============


Top-level keys
--------------

.. include-field:: snapcraft.models.project.Project name

.. include-field:: snapcraft.models.project.Project version

.. include-field:: snapcraft.models.project.Project build_base
    :override-name: base

.. include-field:: snapcraft.models.project.Project compression

.. include-field:: snapcraft.models.project.Project donation

.. include-field:: snapcraft.models.project.Project source_code
    :override-name: source-code

.. include-field:: snapcraft.models.project.Project contact

.. include-field:: snapcraft.models.project.Project issues

.. include-field:: snapcraft.models.project.Project website

.. include-field:: snapcraft.models.project.Project type

.. include-field:: snapcraft.models.project.Project layout

.. include-field:: snapcraft.models.project.Project grade

.. include-field:: snapcraft.models.project.Project architectures

.. include-field:: snapcraft.models.project.Project platforms

.. include-field:: snapcraft.models.project.Project assumes

.. include-field:: snapcraft.models.project.Project hooks

.. include-field:: snapcraft.models.project.Project passthrough

.. include-field:: snapcraft.models.project.Project plugs

.. include-field:: snapcraft.models.project.Project slots

.. include-field:: snapcraft.models.project.Project lint

.. include-field:: snapcraft.models.project.Project epoch

.. include-field:: snapcraft.models.project.Project adopt_info

.. include-field:: snapcraft.models.project.Project system_usernames
    :override-name: system-usernames

.. include-field:: snapcraft.models.project.Project environment

.. include-field:: snapcraft.models.project.Project build_packages
    :override-name: build-packages

.. include-field:: snapcraft.models.project.Project build_snaps
    :override-name: build-snaps

.. include-field:: snapcraft.models.project.Project ua_services
    :override-name: ua-services

.. include-field:: snapcraft.models.project.Project provenance

.. include-field:: snapcraft.models.project.Project components

.. include-field:: snapcraft.models.project.Project apps


App keys
--------

.. include-model:: snapcraft.models.project.App
    :name-prepend: apps.<app name>


Part keys
---------

.. include-model:: craft_parts.parts.PartSpec
    :name-prepend: <part name>
