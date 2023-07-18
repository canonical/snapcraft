.. 20913.md

.. _progressive-releases:

Progressive releases
====================

When a snap :ref:`has been published <releasing-your-app>` and has an established user base, tracks and channels help balance the risk of a release containing unexpected issues against user expectations for new features. This is covered in `Release management <https://snapcraft.io/docs/release-management>`__.

Progressive releases offer an additional strategy to help mitigate the risk of unexpected issues affecting users. They help by making a release available only to a specific percentage of a snap’s user base. This percentage can be initially small, perhaps 10% or 20%, and increases as confidence in a release grows.

To deploy a progressive releases, add an additional argument to the ``snapcraft release`` command, which itself operates on an uploaded revision of a snap:

``snapcraft release <snap-name> <revision> <channel> --progressive <percent>``

The percentage value defines the proportion of devices a progressive release will be deployed to.

.. note:: Snapcraft 7.x or newer is required to use progressive releases.


Creating a progressive release
------------------------------

First, ensure there is a revision of the target snap available on the `Snap Store <https://snapcraft.io/store>`__. See :ref:`Releasing your app <releasing-your-app>` for details on building and uploading locally, or :ref:`Remote build <remote-build>` to create a revision on remote servers using `Launchpad <https://launchpad.net/>`__.

The ``snapcraft revisions`` command can be used to see which revisions have been uploaded and available:

.. code:: bash

   $ snapcraft revisions <snap-name>
   Rev. Uploaded              Arches       Version  Channels
   357  2020-11-03T11:29:13Z  amd64,armhf  4.1255   -
   356  2020-11-03T08:27:23Z  amd64,armhf  4.1249   latest/candidate*
   355  2020-10-26T13:07:16Z  amd64,armhf  4.1246   latest/stable

In the above example output, revision 357 of the snap has been uploaded but has not been released, revision 356 is on the candidate channel and 355 is on the stable channel. To release revision 356 to the stable channel of the default (latest) track, issue the following command:

.. code:: bash

   $ snapcraft release <snap-name> 356 stable

To release revision 356 to the stable channel of the default (latest) track as a progressive release, with 30% deployment, issue the following command:

.. code:: bash

   $ snapcraft release <snap-name> 356 stable --progressive 30

The command above will release the snap to 30% of devices, chosen pseudo-randomly based in part on a hash of their device ID, with the snap installed from its stable channel. This means that roughly every third device will get the update when it next requests a refresh.


Monitoring a release
--------------------

The *snapcraft status* command can be used to monitor the progress of a progressive release:

.. code:: bash

   $ snapcraft status <snap-name>
   Track     Arch    Channel    Version    Revision    Progress
   latest    amd64   stable     4.1246     341         73 → 70%
                                4.1249     355         21 → 30%
                     candidate  ↑          ↑           -
                     beta       -          -           -
                     edge       -          -           -

The above output shows that the progressive release of the *stable* channel has been set to target 30% of devices, with 70% remaining on revision 341 and 30% upgrading to revision 355. The *Progress* column shows the current deployment progress towards those percentages.

If an issue is discovered with a revision deployed as a progressive release, a new revision can be built and uploaded to address the issue without further deployment. When the new revision is itself published as a progressive release to the same channel, the devices that received the earlier release will be prioritised.


Finalising a progressive release
--------------------------------

After the assigned percentage of devices have received the release, there are two ways the snap publisher can proceed:

1. **Re-release the same revision again with a higher percentage** and continue to do so manually, pausing to solicit user reports of any issues, until 100% is reached:

   ``snapcraft release <snap-name> 356 stable --progressive 40``

   When a progressive release reaches 100% (with ``--progressive 100``), a non-progressive release is still required. This is because certain devices may be configured to ignore progressive releases entirely.

2. **Release the same revision non-progressively**. This makes the revision available to 100% of devices with the snap installed:

   ``snapcraft release <snap-name> 356 stable``

After a non-progressive release, a snap will revert to standard `Release management <https://snapcraft.io/docs/release-management>`__ processes and procedures.
