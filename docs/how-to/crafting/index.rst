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
- :ref:`how-to-include-local-files-and-remote-resources`


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


Parts processing
----------------

If you need to manually manipulate a part's files and data during processing, you can
override the lifecycle steps.

- :ref:`how-to-override-the-parts-lifecycle`


Builds
------

While crafting, you'll typically build on your local host. To speed up local builds, you
can cache packages downloaded with APT. When your snap matures or becomes
cross-platform, you can build at scale on Launchpad.

- :ref:`how-to-build-snap-remotely`
- :ref:`how-to-reuse-packages-between-builds`


.. toctree::
    :hidden:

    configure-package-information
    specify-a-base
    select-platforms
    manage-dependencies
    include-local-files-and-remote-resources
    add-a-snap-configuration
    use-layouts
    enable-classic-confinement
    manage-data-compatibility
    override-the-parts-lifecycle
    create-a-component
    reuse-packages-between-builds
    build-snap-remotely
