.. 12442.md

.. _release-management:

Release management
==================

After a snap has been :ref:`created <creating-a-snap>` and :ref:`released <releasing-your-app>` to the `Snap Store <https://snapcraft.io/store>`__, its published revisions can be moved between :ref:`channels <channels>` from both the command line and from the :ref:`Snap Store web UI <using-the-snap-store>`.

Moving a snap between channels helps to manage a user’s expectations in any trade-off between stability in the *stable* channel, and cutting edge features in the *edge* channel (as an arbitrary example). But its also a useful technique for beta testing, or for when a snap needs to revert to a previous revision.

See :ref:`Releasing your app <releasing-your-app>` for details on how to upload and publish a snap if you haven’t done so already.

.. _release-management-1:

Release management
------------------

The web UI’s release management functionality is equivalent to using ``snapcraft release`` on the command line, and both require that you first login to the store.

To access the Snap Store web UI, go to https://snapcraft.io/store and either :ref:`Create a developer account <create-a-developer-account>` or login with your developer account credentials.

After logging in to the Snap Store and selecting a published snap, click the ‘Releases’ tab to access the release management functions.

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/a/ac55ffb51aef79fc53e87a8b880b35b0a46d22d4.png
   :alt: Screenshot_20200828_123936|690x458


The *Releases* page lists which revisions of a published snap will be delivered to users tracking a specific channel.

On the command line, with a developer account already created, enter the following to login:

.. code:: bash

   $ snapcraft login

The command line equivalent to the web UI’s *Releases* page is the output from ``snapcraft status <snap-name>``:

.. code:: bash

   $ snapcraft status opencorsairlink
   Track    Arch     Channel    Version    Revision
   latest   amd64    stable     61d336a    127
                     candidate  61d336a    127
                     beta       61d336a    127
                     edge       61d336a    127
            arm64    stable     46dbf20    95
                     candidate  46dbf20    95
                     beta       46dbf20    95
                     edge       61d336a    128
            armhf    stable     46dbf20    94
                     candidate  46dbf20    94
                     beta       46dbf20    94
                     edge       61d336a    129
   [...]

To move a snap between channels in the web UI, simply drag a revision from one channel to another, or use the cog drop-down menu on a revision and select a destination.

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/d/dd62e5c21cdc9c5b1c42eb8e0fca2b421cfd689c.png
   :alt: Screenshot_20200828_125118|290x295


Click *Save* to make the requested change, or *Revert* to undo the proposed changes.

On the command line, the *release* command takes the snap name, the revision you wish to move and the destination channel (or channels) as its arguments:

.. code:: bash

   $ snapcraft release mysnap 13 candidate

You can also move earlier revisions back into a channel:

.. code:: bash

   $ snapcraft release mysnap 5 beta

See :ref:`channels` for more details on how channels, tracks and branches can be used, or :ref:`Publish to a branch <publish-to-a-branch>` for details on how snap developers can use branches to publish temporary snap releases.


.. _heading--distribution:

Limiting distribution
---------------------

A snap’s distribution can be limited from the Snap Store web UI by two options listed on a snap’s *Settings* page:

-  **Visibility**: controls *who* can see a snap and install a snap. See `Public, Private and Unlisted snaps <https://forum.snapcraft.io/t/public-private-and-unlisted-snaps/19744>`__ for more details.
-  **Distribution**: controls the territories where a snap can be installed or not installed, as outlined below.

The *Distribution* options set whether a snap can be installed in all territories (default), or whether its distribution is either

-  limited to selected territories
-  excluded from selected territories

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/6/6f935ddb3111e3eb98d38f5cb54e47763bac8234.png
   :alt: Screenshot_20200901_125521|690x390


Activating either of the *Selected territories* fields will open a drop-down list of territories from which to choose from. More than one territory can be added.
