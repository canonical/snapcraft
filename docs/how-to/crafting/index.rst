.. _how-to-crafting:

Crafting
========

This section contains guides for putting together your snap and managing its contents.


Basic configuration
-------------------

The essential elements in a snap are the package information, base, and platforms.

- :ref:`how-to-configure-package-information`
- :ref:`how-to-specify-a-base`
- :ref:`how-to-select-platforms`


Software dependencies
---------------------

Much of the work of making a snap is in determining and adding its dependencies.

- :ref:`how-to-manage-dependencies`


User settings and data
----------------------

You can make snap settings available to users, and control its data retention on the
user's machine between versions.

- :ref:`how-to-add-a-snap-configuration`
- :ref:`how-to-manage-data-compatibility`


Confinement
-----------

If your snap needs to make its files available to the host, or it needs less restrictive
permissions with the host, you can extend access with layouts and the confinement level.

- :ref:`how-to-use-layouts`
- :ref:`how-to-enable-classic-confinement`


Data contents
-------------

Snaps can package plain assets.

- :ref:`how-to-include-local-files-and-remote-resources`


Process overrides
-----------------

If you need to manually manipulate a part's files and data during processing, you can
override the lifecycle steps. To speed up the build, you can cache packages downloaded
with APT.

- :ref:`how-to-override-the-parts-lifecycle`
- :ref:`how-to-reuse-packages-between-builds`


.. toctree::
    :hidden:

    configure-package-information
    specify-a-base
    select-platforms
    manage-dependencies
    add-a-snap-configuration
    use-layouts
    manage-data-compatibility
    include-local-files-and-remote-resources
    override-the-parts-lifecycle
    reuse-packages-between-builds
    create-a-component
    enable-classic-confinement
