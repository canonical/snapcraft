.. |app| replace:: Snapcraft
.. |app-command| replace:: snapcraft
.. |artifact| replace:: snap
.. |an-artifact| replace:: a snap
.. |app-min-pro-version| replace:: 9.1
.. |app-link| replace:: https://snapcraft.io/snapcraft

.. _how-to-pack-a-pro-snap:

.. include:: ../../common/craft-application/how-to-guides/pack-a-pro-artifact.rst
    :end-before: .. Split point for downstream docs: Prerequisites onward

Core24 and higher
~~~~~~~~~~~~~~~~~~~

.. include:: ../../common/craft-application/how-to-guides/pack-a-pro-artifact.rst
    :start-after: .. Split point for downstream docs: Prerequisites onward

Core22
~~~~~~~

Core22 snaps use a legacy mechanism to pack Pro-compliant snaps. As a result, it uses
the previous name UA (Ubuntu Advantage) instead of Ubuntu Pro.

Prerequisites
-------------

- An Ubuntu Pro token (https://ubuntu.com/pro)

Identify the required Pro services
----------------------------------

First, determine which Pro services fit your needs:

- ``esm-apps`` or ``esm-infra``: If your goal is to pack a snap for an application
  and include the latest security patches for a base that is no longer under Standard
  Security Maintenance.
- ``fips``, ``fips-updates`` or ``fips-preview``: If you need to deploy a snap in a
  highly regulated environment that processes sensitive data.

The desired Pro services must be available. On a system with your Pro token
attached, run ``pro status`` and check the ``ENTITLED`` column for available
services. The Ubuntu Pro Client documentation has `further information
<https://documentation.ubuntu.com/pro-client/en/v32/explanations/which_services/>`__ on
each service.

To enable the desired Pro services when building your snap, add a ua-services key to
your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    ua-services:
      - esm-apps
      - esm-infra

Pack the snap
-------------

Now you can pack the snap with the desired services by passing the Pro token:

.. code-block:: bash
    :substitutions:

    snapcraft pack --ua-token <pro-token> --enable-experimental-ua-services

The Pro token can also be provided as an environment variable:

.. code-block:: bash
    :substitutions:

    SNAPCRAFT_UA_TOKEN=<token> snapcraft pack --enable-experimental-ua-services

As Pro enablement is an experimental feature, ``--enable-experimental-ua-services`` is
needed in either case.

Snapcraft will automatically attach the Pro subscription and enable the requested
services in the build environment while packing the snap.
