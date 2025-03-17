.. _how-to-check-a-snaps-status:

Check a snap's status
=====================

With Snapcraft, snap authors can easily monitor the status of their published snaps from
the command-line. This guide highlights commands for checking the status of a snap's
releases as well as its visibility on the Snap Store.


Check the status of releases
----------------------------

Run the ``snapcraft status`` command followed by a snap's name to check the status of
its releases.

.. terminal::
    :input: snapcraft status <snap-name>

    Track    Arch     Channel    Version    Revision
    latest   amd64    stable     61d336a    127
                      candidate  61d336a    127
                      beta       61d336a    127
                      edge       61d336a    127
             arm64    stable     46dbf20    95
                      candidate  46dbf20    95
                      beta       46dbf20    95
                      edge       61d336a    128

If a progressive release is in progress, ``snapcraft status`` will also display the
current deployment progress towards the percentages provided when the progressive
release was created. For example, a progressive release into the stable channel with a
deployment target of 30% would result in output similar to:

.. terminal::
    :input: snapcraft status <snap-name>

    Track     Arch    Channel    Version    Revision    Progress
    latest    amd64   stable     4.1246     341         73 → 70%
                                 4.1249     355         21 → 30%
                      candidate  ↑          ↑           -
                      beta       -          -           -
                      edge       -          -           -


Check the visibility of the snap
--------------------------------

Run the ``snapcraft names`` command to check the visibility status of all snaps
associated with your developer account.

.. terminal::
    :input: snapcraft names

    Name             Since                 Visibility    Notes
    cameractrls      2022-11-28T18:15:44Z  public        -
    domoticz         2020-01-17T17:21:43Z  public        -
