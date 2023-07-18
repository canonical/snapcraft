.. 35613.md

.. _revisions:

Revisions
=========

A snap’s *revision* is a number assigned automatically by the :term:`Snap Store` with each snap upload, giving each snap binary a unique identity within, and across, its :term:`channels`.

The revision number increments with each new upload. But this number is arbitrary, and only used to differentiate between uploads.

Neither the revision number (nor its version) enforce an order of release. The local system will attempt to install whatever snap is recommended by the publisher in the channel being tracked.


Viewing revision numbers
------------------------

The output to ``snap info <snapname>`` includes the revision number for each snap in each track and channel as a number in brackets after the publishing date:

::

   channels:
     latest/stable:    2.59.4                 2023-05-31 (19361) 55MB -
     latest/candidate: 2.59.5                 2023-06-09 (19457) 55MB -
     latest/beta:      2.59.5                 2023-05-27 (19457) 55MB -
     latest/edge:      2.59.5+git955.gc9310cc 2023-06-13 (19535) 42MB -
   installed:          2.59.1+git798.g5f761c7            (19131) 42MB snapd

In the above example output, the latest/edge snap has a revision of ``19535`` and is the most recent published revision of the snap. If the snap is installed, the final line of ``snap info`` output includes the installed revision (``19131``, above).

The ``snap list`` command includes a column for each installed snap’s revision number, labelled ``Rev``:

::

   Name         Version  Rev    Tracking       Publisher             Notes
   alacritty    0.8.0    46     latest/stable  snapcrafters✪         classic
   blender      3.5.0    3486   latest/stable  blenderfoundation✓    classic
   chromium     112.0    2424   latest/stable  canonical✓            -
   ffmpeg       4.3.1    1286   latest/stable  snapcrafters✪         -


Revision package management
---------------------------

The :term:`Snap Store` caches several older revisions of every snap, as does the local system. By default, 2 revisions are stored locally, while :term:`Ubuntu Core` systems store 3. These defaults can be changed with the `refresh-retain <https://snapcraft.io/docs/managing-updates#revisions-heading--refresh-retain>`__ system option.

The snap ``install``, ``refresh`` and ``download`` commands can operate on these available revisions with an optional ``--revision`` argument.

.. code:: bash

   snap <command> <snap-name> --revision <revision-number>

To install revision number *27* of the hello-world snap, for example, run the following command:

.. code:: bash

   $ snap install hello-world --revision 27
   hello-world 6.3 from Canonical✓ installed

The hello-world snap could be refreshed to revision number *28* with the following command:

.. code:: bash

   $ snap refresh hello-world --revision 28
   hello-world 6.3 from Canonical✓ refreshed

The revision number of the snap being operated upon will appear in the output during these operations.

`Release management <https://snapcraft.io/docs/release-management>`__ details how a snap developer can publish or promote specific revisions of their snap.


Data management
---------------

The revision identity is used as a reference for revision-specific data. As described in :ref:`Data locations <data-locations>`, revision-specific data for a snap is stored in a either system-wide location, or a user-specific home location:

-  **SNAP\_ DATA**: ``/var/snap/<snap name>/<revision number>``\  This location is also used to store data, mostly information utilised by background application and services, for logging, and other tasks that require persistence between snap launches.

-  **SNAP_USER_DATA** : ``/home/<username>/snap/<snap name>/<revision>``\  This location contains any user data that the snap writes to its own home. This is *in contrast* to what the Linux user would consider *their* home, although the location itself will be in the user’s home directory.

   It is important to note this distinction, because it can be useful, and even important when users decide to perform maintenance operations with their snaps (like removal). By default, every snap will use a symlink *current* , pointing to the latest available revision.

When you move from one revision to the next, the revision-specific contents of **SNAP_DATA** and **SNAP_USER_DATA** are copied into new directories for the new revision. This includes moving from a higher revision number to a lower revision number (because revision numbers are arbitrary).

Revision-specific directories are retained inline with the `refresh-retain <https://snapcraft.io/docs/managing-updates#revisions-heading--refresh-retain>`__ system option.

Other than the contents of the common directories, a `Snapshot <https://snapcraft.io/docs/snapshots>`__ stores only the data associated with the currently installed revision. See `What a snapshot stores <https://snapcraft.io/docs/snapshots#revisions-heading--what-is-stored>`__ for more details.
