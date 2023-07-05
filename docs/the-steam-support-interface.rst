.. 30990.md

.. _the-steam-support-interface:

The steam-support interface
===========================

The ``steam-support`` interface has been developed specifically to help Valve’s Steam client configure `pressure-vessel <https://gitlab.steamos.cloud/steamrt/steam-runtime-tools/-/tree/master/pressure-vessel>`__ containers from the `Steam snap <https://snapcraft.io/steam>`__.

Only the Steam snap may establish this interface. See `Introducing early access to the Steam snap <https://discourse.ubuntu.com/t/introducing-early-access-to-the-steam-snap/28082>`__ for more details.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-steam-support-interface-heading--dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

Code examples
-------------

The source code for the Steam snap: https://github.com/canonical/steam-snap

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/steam_support_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/steam_support.go
