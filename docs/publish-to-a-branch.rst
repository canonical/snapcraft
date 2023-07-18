.. 29544.md

.. _publish-to-a-branch:

Publish to a branch
===================

A branch is an optional finer subdivision of a :term:`channel` which allows for the creation of a temporary and short-lived sequence of snaps. They’re typically used by snap developers to push fixes or temporary experimental builds on-demand.

Branch names convey their purpose, such as ``fix-for-bug123``, but the name isn’t exposed in the normal way, such as with ``snap info``. Instead, they can only be installed by someone who knows the branch name, and this is usually only shared by the snap developer to test a specific fix or release.

After 30 days with no further updates, a branch will be closed automatically. The replacement snap will then be chosen from the next most conservative risk-level of the same track. For example, *beta/fix-for-bug123* will fall back to *beta* after the *fix-for-bug123* branch is closed.


Creating a branch
-----------------

To publish a built and tested snap to a new branch, the snap developer can use the same *snapcraft upload* process used to publish the snap, but with a full :term:`channel` description that includes track, risk-level and a new arbitrary branch name:

.. code:: bash

   snapcraft upload <snap-name.snap> --release=<track>/<risk>/<branch>

The branch name cannot contain “``_``” or “``/``” character.

See :ref:`Releasing your app <releasing-your-app>` for more general details on publishing a snap.

Many snaps will have only a *latest* track, while others may use different tracks for each supported release, such as `Node.js <https://snapcraft.io/node>`__ or separate tracks for stable and insider builds, such as `Skype <https://snapcraft.io/skype>`__. These details can be retrieved with the ``snap info`` command:

.. code:: bash

   $ snap info qvge
   name:      qvge
   [...]
   channels:
     latest/stable:    0.6.0-1 2021-08-28  (45) 103MB -
     latest/candidate: 0.6.1   2021-11-18  (81) 101MB -
     latest/beta:      0.6.1   2021-11-18  (81) 101MB -
     latest/edge:      0.6.3   2022-02-28 (225) 101MB -

The above example shows a snap with only a *latest* track. To publish a locally built snap to its *latest/beta* channel with a new branch called ``fix-test-062``, you would run the following command:

.. code:: bash

   $ snapcraft upload qvge_0.6.2_amd64.snap --release=latest/beta/fix-test-062
   Preparing to upload 'qvge_0.6.2_amd64.snap'.
   After uploading, the resulting snap revision will be released to 'latest/beta/fix-test-062' when it passes the Snap Store review.
   Running the review tools before pushing this snap to the Snap Store.
   Generating delta for 'qvge_0.6.2_amd64.snap'.
   Pushing 'qvge_0.6.2_amd64.snap.xdelta3' [=================================================================================] 100%
   Processing...|
   released
   Revision 230 of 'qvge' created.
   Track    Arch    Channel            Version    Revision    Expires at
   latest   amd64   stable             0.6.0-1    45
                    candidate          0.6.1      81
                    beta               0.6.1      81
                    edge               0.6.3      225
                    beta/fix-test-062  0.6.2      230         2022-05-13T11:10:26Z

The above shows that the new branch now exists under the latest track, beta branch. It can be confirmed by the snap developer with the ``snapcraft status`` command, but the branch will not appear in the output to ``snap info``:

.. code:: bash

   $ snapcraft status qvge
   Track    Arch     Channel            Version    Revision    Expires at
   latest   amd64    stable             0.6.0-1    45
                     candidate          0.6.1      81
                     beta               0.6.1      81
                     edge               0.6.3      225
                     beta/fix-test-062  0.6.2      230         2022-05-13T11:10:26Z
   $ snap info qvge
   name:      qvge
   [...]
   channels:
     latest/stable:    0.6.0-1 2020-08-28  (45) 103MB -
     latest/candidate: 0.6.1   2020-11-18  (81) 101MB -
     latest/beta:      0.6.1   2020-11-18  (81) 101MB -
     latest/edge:      0.6.3   2021-09-28 (225) 101MB -


Installing a snap from a branch
-------------------------------

To install a snap from a branch, the user needs to know its name. This is typically shared by the developer, either through whatever issue tracking system might be used by the project, or shared informally via a forum post or message.

When you know the branch name, the snap can be installed with the ``snap install <snap-name> --channel`` command, followed by the full channel description. To install a snap called *qvge* from its ``beta/fix-test-062`` branch, for instance, you’d type the following:

.. code:: bash

   $ snap install qvge --channel beta/fix-test-062
   qvge (beta/fix-test-062) 0.6.2 installed

If the snap is already installed, replace **install** with **refresh**.

After 30 days with no further updates, a branch will be closed automatically. The replacement snap will then be chosen from the next most conservative risk-level of the same track. For example, *beta/fix-test-061* will fall back to whatever snap is provided by *beta* after the fix-test-061 branch is closed.


Promoting a snap from a branch
------------------------------

If a snap in a branch proves stable and fixes whatever issue necessitated the branch release, it can be promoted to another channel just like any other snap.

A branch is visible on a snap’s ‘Release’ page in the `Snap Store web UI <https://snapcraft.io/snaps>`__, from where it can be promoted to a different channel just like any other release:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/f/f872a50bf0a3db7e999260fea035fd4b32fa920f.png
   :alt: image|690x367


A snap from a branch can also be promoted to another channel using *snapcraft* on the command line:

.. code:: bash

   $ snapcraft release qvge 230 beta
   Track    Arch    Channel            Version    Revision    Expires at
   latest   amd64   stable             0.6.0-1    45
                    candidate          0.6.1      81
                    beta               0.6.2      230
                    edge               0.6.2      231
                    beta/fix-test-062  0.6.2      230         2022-05-13T11:10:26Z

After a snap has been promoted, the branch will remain in-place until its expiry.

For more details on promoting snaps to different channels, see `Release management <https://snapcraft.io/docs/release-management>`__.


Setting a default track
-----------------------

All snaps have a default track. When not specified explicitly, a snap is installed from the default track and without the snap publisher specifying otherwise, the default track is called *latest*

A default track can be specified manually with *snapcraft* when a snap has more than one track:

::

   snapcraft set-default-track <snap-name> <default-track-name>

When no track is specified at install time, an *implicit track* install will install from ``<default-track-name>`` instead of ``latest``:

.. code:: bash

   snap install <snap-name>

Note that the default track does NOT remove latest; latest is never a pointer to another track and remains available:

.. code:: bash

   snap install <snap-name> --channel=latest
