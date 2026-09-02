.. _how-to-manage-revisions-and-releases:

Manage revisions and releases
=============================

Snapcraft and snap stores offer ways to maintain and control the history of your snap's
revisions and releases. Snaps can have multiple concurrent releases on different
:ref:`channels <reference-channels>`, and stores provide capabilities for progressive
releases, interim revisions, and moving revisions between channels.


Publish a revision
------------------

There are two methods to publish a revision. The first separates the upload and publish
steps, the second performs both at the same time.


Publish after uploading
~~~~~~~~~~~~~~~~~~~~~~~

If you want to first upload a snap before placing it in a channel, you can upload it
first, then publish it.

After building the revision of your snap, first upload it:

.. code-block:: bash

    snapcraft upload <snap-revision>.snap

The output contains the revision number of the snap, which you'll use later.

When you're ready to publish the revision to a channel, run:

.. code-block:: bash

    snapcraft release <snap-name> <revision> <channel>


Publish and upload at the same time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Most of the time, snap authors know the destination channel of a snap update, and so
they upload and publish it simultaneously. To do so, run:

.. code-block:: bash

    snapcraft upload <snap-revision>.snap --release <channel>

If you need to publish the same revision to multiple channels, list multiple channel
names in the argument, separated by commas (,):

.. code-block:: bash

    snapcraft upload <snap-revision>.snap --release <channel-a>,<channel-b>


Publish a branch
~~~~~~~~~~~~~~~~

Publishing a :ref:`branch <reference-channels-branch>` is similar process, but you must
provide a full channel name, meaning it must include the track, risk, and the new branch
name. A branch name can't contain underscores (_) or forward slashes (/).

As with regular publication, you can publish after uploading the revision:

.. code-block:: bash

    snapcraft upload <snap-revision>.snap
    snapcraft release <snap-name> <revision> <track>/<risk>/<branch>

Or simultaneously:

.. code-block:: bash

    snapcraft upload <snap-revision>.snap --release <track>/<risk>/<branch>


Promote a revision
------------------

After a revision is published, you can move it to another channel.

Moving a snap revision between channels helps manage user expectations and the trade-off
between stability and cutting-edge features. The two default channels, stable and edge,
are reflections of these two expectations. In fact, promoting from edge to stable is the
recommended and most common lifecycle for a snap revision.

Promoting and demoting revisions is also a useful progression for beta testing, or for
when a snap needs to revert to an older version as a response to a security concern.

You can promote a published revision to another channel with:

.. code-block:: bash

    snapcraft release <snap-name> <revision> <new-channel>

Replace ``<revision>`` with the revision's unique number. You can obtain a revision
number with the ``snapcraft revisions`` command.

For example, if you were the Firefox maintainer and wanted to make an edge revision
generally available on the stable channel, you'd run:

.. code-block:: bash

    snapcraft release firefox 531 stable


Deliver a progressive release
-----------------------------

Progressive releases are a strategy to mitigate the risk of unexpected issues
originating from snap revision. They help by making a release available only to a
specific percentage of a snap's user base. This percentage can be initially small,
perhaps 10% or 20%, and increases as confidence in the release grows.


Start the release
~~~~~~~~~~~~~~~~~

First, ensure there is a revision of the target snap available on the Snap Store. See
:ref:`explanation-remote-build` to create a revision on remote servers using Launchpad.

Take an example snap that has revision 356 on a channel called *candidate*. To
progressively release revision 356 to the stable channel with 30% deployment, you would
run:

.. code-block:: bash

    snapcraft release <snap-name> 356 stable --progressive 30

The revision would release to 30% of devices that installed the snap on the stable
channel, chosen at random. Roughly one out of three devices will apply the update when
they next refresh the snap.


Close the release
~~~~~~~~~~~~~~~~~

After the assigned percentage of devices have all applied a progressive release, there
are two ways you can complete the release:

1. Re-release the revision with a higher percentage. Continuing with our earlier
   example:

   .. code-block:: bash

        snapcraft release <snap-name> 356 stable --progressive 40

   You can continue continue to do so manually, until the release reaches 100%
   coverage. These manual cycles provide you with openings to solicit user feedback and
   reports.

   When a progressive release reaches 100% (with ``--progressive 100``), a
   non-progressive release is still required. This is because certain devices may be
   configured to ignore progressive releases entirely.

2. Republish the revision as a standard release. Doing so makes the revision available
   to 100% of devices with the snap installed. With our ongoing example:

   .. code-block:: bash

        snapcraft release <snap-name> 356 stable

After a non-progressive release, a snap will revert to standard release lifecycle.


Change the default track
------------------------

All snaps have a default :ref:`track <reference-channels-track>` called **latest**.
Unless otherwise specified, users install the most recent release on the default track.

If you've received `approval for a new track
<https://forum.snapcraft.io/t/simplified-track-request-process-for-snaps-with-predictable-cadence/3136>`_,
you can make it your snap's default track. Changing the default track doesn't remove the
latest track -- it's always available, even if you don't manage it.

To change the default track to a custom track, run:

.. code-block:: bash

    snapcraft set-default-track <snap-name> <custom-name>

When users install your snap without specifying a channel, the new default track is
selected.
