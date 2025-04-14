.. _how-to-check-a-snaps-status:

Check a snap's status
=====================

From the terminal, you can check the status of your published snap, its releases, and
its visibility.

.. If you prefer to monitor your snaps in a web browser, you can `check your snap's
   status on the Snap Store <>`_.


Log in to the Snap Store
------------------------

First, you must log in to the Snap Store with your Ubuntu One account. If you don't
already have an account, create one `here <https://login.ubuntu.com/>`_.

To log in to your account in the terminal, run:

.. code-block:: bash

    snapcraft login


Check the status
----------------

There are multiple status commands, depending on your focus.

The base status check is oriented on the channels that you've published. To check the
channel status, run:

.. code-block:: bash

    snapcraft status <snap-name>

If your account authored the snap, the output lists the current revisions by channel.

.. terminal::

    Track    Arch     Channel    Version    Revision
    latest   amd64    stable     61d336a    127
                      candidate  61d336a    127
                      beta       61d336a    127
                      edge       61d336a    127
             arm64    stable     46dbf20    95
                      candidate  46dbf20    95
                      beta       46dbf20    95
                      edge       61d336a    128

If a progressive release is in progress, the output also shows how close the release is
to its target deployment percentage. For example, a progressive release of the stable
channel with a deployment target of 30% would produce output like the following.

.. terminal::

    Track     Arch    Channel    Version    Revision    Progress
    latest    amd64   stable     4.1246     341         73 → 70%
                                 4.1249     355         21 → 30%
                      candidate  ↑          ↑           -
                      beta       -          -           -
                      edge       -          -           -

If you want to filter for particular architectures, add the ``--arch`` argument and
specify the ones you want:

.. code-block:: bash

    snapcraft status <snap-name> --arch amd64

It's sometimes more helpful to check the status of revisions, for example after
submitting a build with multiple architectures. To view a chronological list of all your
snap's revisions, run:

.. code-block:: bash

    snapcraft list-revisions <snap-name>

The output is ordered by revision number, and contains the revisions for all
architectures.

.. terminal::

    Rev.    Uploaded              Arches    Version              Channels
    175     2025-03-13T09:26:16Z  riscv64   v0.29.0-6-gc772624   latest/edge*
    174     2025-03-13T08:04:12Z  armhf     v0.29.0-6-gc772624   latest/edge*
    173     2025-03-13T08:03:21Z  ppc64el   v0.29.0-6-gc772624   latest/edge*
    172     2025-03-13T08:02:12Z  arm64     v0.29.0-6-gc772624   latest/edge*
    171     2025-03-13T08:01:48Z  s390x     v0.29.0-6-gc772624   latest/edge*
    170     2025-03-13T07:59:46Z  amd64     v0.29.0-6-gc772624   latest/edge*
    169     2025-03-11T09:24:42Z  riscv64   v0.29.0-5-g9c64eb4   latest/edge
    168     2025-03-11T08:04:13Z  ppc64el   v0.29.0-5-g9c64eb4   latest/edge
    167     2025-03-11T08:04:12Z  arm64     v0.29.0-5-g9c64eb4   latest/edge
    166     2025-03-11T08:03:13Z  armhf     v0.29.0-5-g9c64eb4   latest/edge
    165     2025-03-11T08:02:12Z  s390x     v0.29.0-5-g9c64eb4   latest/edge
    164     2025-03-11T08:00:13Z  amd64     v0.29.0-5-g9c64eb4   latest/edge
    163     2025-03-02T09:13:12Z  riscv64   v0.28.2-1-geef628d   latest/edge
    162     2025-03-02T08:00:45Z  ppc64el   v0.28.2-1-geef628d   latest/edge
    161     2025-03-02T07:59:38Z  arm64     v0.28.2-1-geef628d   latest/edge
    160     2025-03-02T07:58:30Z  armhf     v0.28.2-1-geef628d   latest/edge
    159     2025-03-02T07:57:11Z  s390x     v0.28.2-1-geef628d   latest/edge
    158     2025-03-02T07:55:11Z  amd64     v0.28.2-1-geef628d   latest/edge
    ...

Like with the channel status, you can filter the revision status for particular
architectures with the ``--arch`` argument:

.. code-block:: bash

    snapcraft list-revisions <snap-name> --arch amd64


Check the public visibility of your snaps
-----------------------------------------

To check the public visibility of every snap you registered on the store, run:

.. code-block:: bash

    snapcraft names

This returns the name, registration date, and visibility of all snaps associated with
the current account.

.. terminal::
    :input: snapcraft names

    Name             Since                 Visibility    Notes
    cameractrls      2022-11-28T18:15:44Z  public        -
    domoticz         2020-01-17T17:21:43Z  public        -
