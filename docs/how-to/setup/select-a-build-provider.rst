.. _how-to-select-a-build-provider:

Select a build provider
=======================

Depending on the core version and platform, there are different default build
providers and multiple ways to change providers. The following tables are
ranked by precedence across cores, so for example a provider chosen by the
first method overrides the second and third method, and so on.

Consult the following tables for different methods of overriding the build
provider.


core22 and higher override methods
----------------------------------

For core22 and higher, LXD is the default provider on Linux, and Multipass is
the default on macOS and Windows.

.. list-table::
  :header-rows: 1
  :widths: 1 1 1

  * - Provider override for core22 and higher
    - Priority
    - Command
  * - Command-line argument
    - 1
    - ``--use-lxd``
  * - Environmental variable
    - 2
    - ``SNAPCRAFT_BUILD_ENVIRONMENT=<provider-name>``
  * - Snap configuration of Snapcraft
    - 3
    - ``snap set snapcraft provider=<provider-name>``


core20 override methods
-----------------------

For core20, Multipass is the default provider on all platforms.

.. list-table::
  :header-rows: 1
  :widths: 1 1 1

  * - Provider override for core20
    - Priority
    - Command
  * - Command-line argument
    - 1
    - ``--provider=<provider-name>``
  * - Environmental variable
    - 2
    - ``SNAPCRAFT_BUILD_ENVIRONMENT=<provider-name>``
  * - Command-line argument
    - 3
    - ``--use-lxd``
  * - Snap configuration of Snapcraft
    - 4
    - ``snap set snapcraft provider=<provider-name>``


Select through the snap configuration
-------------------------------------

The snap configuration method mentioned in the previous sections is a `feature
of snapd <https://snapcraft.io/docs/configuration-in-snaps>`_.

Like most snaps, Snapcraft isn't installed with any default configuration
values. As a result, you won't be able to check the provider programmatically
if you haven't already set it on the system.

To set the provider, run:

.. code:: bash

  snap set snapcraft provider=<provider-name>

Where ``<provider-name>`` can be ``lxd`` or ``multipass``.

To check the provider after it's been set, run:

.. code:: bash

  snap get snapcraft provider

To unset the provider, run:

.. code:: bash

  snap unset snapcraft provider
