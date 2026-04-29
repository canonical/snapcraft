.. _how-to-crafting:

Crafting
========

This section contains guides for putting together your snap and managing its contents.

The most basic and least variable elements you define for your snap are the package
information, base, and platforms.

- :ref:`how-to-configure-package-information`
- :ref:`how-to-specify-a-base`
- :ref:`how-to-select-platforms`

Much of defining a working part in a snap is in determining and adding its dependencies.

- :ref:`how-to-manage-dependencies`

You can make snap settings available to users, and control its data retention on the
user's machine between versions.

- :ref:`how-to-add-a-snap-configuration`
- :ref:`how-to-manage-data-compatibility`

If your snap needs to make its files available to the host, or it needs less restrictive
permissions with the host, you can extend access with layouts and the confinement level.

- :ref:`how-to-use-layouts`
- :ref:`how-to-enable-classic-confinement`

Snaps can contain plain assets that aren't compiled at build-time.

- :ref:`how-to-include-local-files-and-remote-resources`

If you need to transfer or modify data within the parts lifecycle, you can override the
individual steps. To speed up the build, you can cache packages downloaded with APT.

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
