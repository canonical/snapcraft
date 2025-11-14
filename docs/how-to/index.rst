.. _how-to-guides:

How-to guides
=============

These pages provide directions for completing tasks and solving problems with Snapcraft.


Setup
-----

You can install Snapcraft on any Linux distribution supporting snapd. After
installation, choose from different back-end build providers.

- :ref:`how-to-set-up-snapcraft`
- :ref:`how-to-select-a-build-provider`


Crafting
--------

The majority of the work you can do with Snapcraft is putting together a buildable snap,
also known as *crafting* a snap. These tasks range from the essentials, such as
choosing the target CPU architectures, to the advanced, such as configuring layouts.

- :ref:`how-to-configure-package-information`
- :ref:`how-to-specify-a-base`
- :ref:`how-to-select-architectures`


Integrations and extensions
---------------------------

Snapcraft accommodates different languages, build systems, and frameworks. These guides
cover how to package software written with them.

- :ref:`how-to-integrations`
- :ref:`how-to-extensions`


Publishing
----------

Once your snap is built, it's ready to be distributed to the Snap Store or private
stores. You can enact a versioning plan for your snap by managing its release cycle and
monitoring its installation metrics.

- :ref:`how-to-register-a-snap`
- :ref:`how-to-manage-revisions-and-releases`
- :ref:`how-to-get-snap-metrics`


Debugging
---------

Like all software, snaps can have build and dependency errors. Snapcraft and its
companion tools have features for identifying and resolving such issues.

- :ref:`how-to-debug-a-snap`
- :ref:`how-to-use-the-library-linter`
- :ref:`how-to-use-the-classic-linter`
- :ref:`how-to-use-the-metadata-linter`
- :ref:`how-to-debug-with-gdb`


Change bases
------------

With each major release of Snapcraft, snap builds gain new features and capabilities,
and lose others. When migrating from one version and base to a new one, you might need
to reconfigure your project as described by these guides.

- :ref:`how-to-change-bases`

.. toctree::
    :hidden:

    set-up-snapcraft
    select-a-build-provider
    crafting/index
    integrations/index
    extensions/index
    publishing/index
    debugging/index
    change-bases/index
