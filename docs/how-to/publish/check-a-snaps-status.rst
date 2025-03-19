.. _how-to-check-a-snaps-status:

Check a snap's status
=====================

From the terminal, you can check the status of your published snap, its releases, and
its visibility.

.. If you prefer to monitor your snaps in a web browser, you can `check your snap's
   status on the Snap Store <>`_.

Log in to the Snap Store
------------------------

First, you must log in to the Snap Store with your Ubuntu One account. If you do not
already have an account, create one `here <https://login.ubuntu.com/>`_.

To log in to your account in the terminal, run:

.. code-block:: yaml

    snapcraft login


Check the status of releases
----------------------------

A snap's status is comprised of all its releases. When you check the status, the store
returns the state of all channels, tracks, and architectures that you've published for.

To check the status of a snap, run:

.. code-block:: yaml

    snapcraft status <snap-name>

If your account is the author of the selected snap, the output contains details similar
to the following example.

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

If a progressive release is in progress, the output also contains the current progress
toward its target deployment percentage. For example, a progressive release of the
stable channel with a deployment target of 30% would produce output similar to:

.. terminal::
    :input: snapcraft status <snap-name>

    Track     Arch    Channel    Version    Revision    Progress
    latest    amd64   stable     4.1246     341         73 → 70%
                                 4.1249     355         21 → 30%
                      candidate  ↑          ↑           -
                      beta       -          -           -
                      edge       -          -           -


Check the public visibility of your snaps
-----------------------------------------

To check the public visibility of all the snaps you registered on the store, run:

.. code-block:: yaml

    snapcraft names

This returns the name, registration date, and visibility of all snaps associated with
the current account.

.. terminal::
    :input: snapcraft names

    Name             Since                 Visibility    Notes
    cameractrls      2022-11-28T18:15:44Z  public        -
    domoticz         2020-01-17T17:21:43Z  public        -
