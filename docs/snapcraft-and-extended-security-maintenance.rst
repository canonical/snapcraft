.. 24297.md

.. _snapcraft-and-extended-security-maintenance:

Snapcraft and Extended Security Maintenance
===========================================

On the 30th April 2021, `Ubuntu 16.04 LTS (Xenial Xerus) <https://releases.ubuntu.com/16.04/>`__ reached the end of its five years of standard support and entered the `Extended Security Maintenance <https://ubuntu.com/security/esm>`__ (ESM) phase.

The former default :ref:`base snap <base-snaps>` when building snaps with :ref:`Snapcraft <snapcraft-overview>` was ``core``, which uses Ubuntu 16.04 LTS as its build and runtime environment. This has since been superseded by ``core18`` and ``core20``, based on Ubuntu 18.04 LTS and Ubuntu 20.04 LTS respectively, which are now the default and recommended build and runtime environments for new snaps.

However, there are still some snaps built using Ubuntu 16.04 LTS, either explicitly by including ``core`` in their :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`, or implicitly by not defining any base at all.

To remain secure and supported, these snaps will need to either:

-  migrate the snap to a later supported base
-  use the Extended Security Maintenance (ESM) programme

See :ref:`Migrating between bases <migrating-between-bases>` for help switching to a newer base, and see below for details on Snapcraft changes and using ESM when building core-based snaps.


Changes to Snapcraft
--------------------

**16.04-based snaps will continue to work**

There will be no immediate impact to either Snapcraft developers or snap users. Snaps will continue to work and your users will be able to continue running them. ESM only becomes relevant as packages need to be updated and patched.

If you’re building snaps locally or through your own CI/CD system:

* Snapcraft now has two :term:`tracks`: ``4.x`` and ``latest`` Both tracks will be maintained in parallel until the release of Snapcraft 5, after which ``4.x`` will be maintained for ESM compatibility.
* cores will be relabelled:

- ``core`` becomes *ESM base*
- ``core18`` and ``core20`` become *LTS bases*.

  * Snap developers and publishers using the ESM base **will not** be able to use Snapcraft 5 or later. A notification will inform developers to use the ``4.x`` track.


Extended Security Maintenance
-----------------------------

The Extended Security Maintenance (ESM) programme extends Canonical’s LTS commitment of providing security updates to the Ubuntu base, and several other critical components, by a few more years. With ESM, Ubuntu 16.04 LTS gains three extra years of security updates, shifting its end of life date to `April 2024 <https://ubuntu.com/security/esm>`__.

Until April 2024, users with an `Ubuntu Advantage <https://ubuntu.com/advantage>`__ (UA) subscription will continue to benefit from security updates for Ubuntu 16.04 LTS, and this includes snap publishers.

**Ubuntu Advantage subscriptions are available for free to individual developers and community members.**

To continue using an ESM base with local and on-premise builds, snap publishers and developers will need to obtain UA tokens. These tokens are free for all community users, for up to three machines, and up to 50 machines for Ubuntu members.

Visit `ubuntu.com/advantage <https://ubuntu.com/advantage>`__ to create an account, or login, and retrieve a token. The token itself can be accessed by expanding the *Machines* drop-down button for the UA subscription you wish to use and copying its value, shown as ``<ua-token`` below:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/optimized/2X/5/58e2e7e29918993f259a25b95d67bc51594a3410_2_616x500.png
   :alt: image|616x500



Using the Ubuntu Advantage token
--------------------------------

The ``--ua-token`` argument is used with the :ref:`snapcraft <snapcraft-overview>` command to specify an Ubuntu Advantage token when building a snap (requires :ref:`Snapcraft 4.7+ <release-notes-snapcraft-4-7>`):

.. code:: bash

   snapcraft --ua-token <ua-token>

The build environment needs the ``ua`` command installed, provided by the ``ubuntu-advantage-tools`` package, and this package is automatically installed by snapcraft.

When the snapcraft command is run, the provisioning of the ESM packages will be done seamlessly in the background.

.. code:: text

   Launching a container.
   Waiting for container to be ready
   Waiting for network to be ready...
   Attaching specified UA token...
   Enabling default service esm-apps
   Updating package lists
   UA Apps: ESM enabled
   Enabling default service esm-infra
   Updating package lists
   UA Infra: ESM enabled
   This machine is now attached to 'UA Applications - Essential (Virtual)'

   SERVICE       ENTITLED  STATUS    DESCRIPTION
   esm-apps      yes       enabled   UA Apps: Extended Security Maintenance (ESM)
   esm-infra     yes       enabled   UA Infra: Extended Security Maintenance (ESM)
   fips          yes       n/a       NIST-certified FIPS modules
   fips-updates  yes       n/a       Uncertified security updates to FIPS modules
   livepatch     yes       n/a       Canonical Livepatch service

   NOTICES
   Operation in progress: ua attach

   Enable services with: ua enable <service>

                   Account:
              Subscription: UA Applications - Essential (Virtual)
               Valid until: 3999-12-31 00:00:00
   Technical support level: essential
   [...]
   Detaching specified UA token...


Remote and Snapcraft build services
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In addition to running *snapcraft* locally, or inside a CI system, snaps can also be built using :ref:`remote build on Launchpad <remote-build>` and our `Snapcraft Build Service <https://snapcraft.io/build>`__. Both of these services will continue working as before.

Launchpad will continue to build for the ESM base without restrictions. It will use the Snapcraft 4.x track for these builds. Similarly, the remote build feature will continue working as before.


GitHub Actions
~~~~~~~~~~~~~~

The GitHub `Snapcraft Build Action <https://github.com/snapcore/action-build>`__ can be used to automatically build a snap. Support for ESM builds via a UA token can be enabled by following https://github.com/snapcore/action-build#ua-token.
