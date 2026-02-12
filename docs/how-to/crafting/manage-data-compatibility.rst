.. _how-to-manage-data-compatibility:


Manage data compatibility
=========================

By defining an epoch in a snap's project file, snap authors can control how users
receive a snap release when its data format is incompatible with older versions.


Define an epoch for a snap
--------------------------

When a new snap release breaks data compatibility with older versions, snap authors can
increment the value of the ``epoch`` key in the snap's project file. This will stop
users of older versions from automatically refreshing to the newer release.

By default, snaps have an epoch of ``0``. If a new version were to break data
compatibility with snaps of this epoch, the new version's ``epoch`` key should be
incremented as follows:

.. code-block:: yaml
    :caption: snapcraft.yaml

    epoch: 1

This will preserve the functionality of older versions until they can be safely
transitioned to the newer epoch.


Update a previous epoch
-----------------------

Once a new epoch is defined, you can still push updates to users of a previous epoch by
editing the value of the project's ``epoch`` key to reflect the target epoch. For
example, if you want to push an update to users of epoch 0 after releasing epoch 1,
update your project file to include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    epoch: 0


Transition to a new epoch
-------------------------

Once a strategy is in place to migrate data from previous epochs to a newer one, you can
append an asterisk (*) to the value of the ``epoch`` key to indicate its compatibility with
the data formats of previous epochs.

For example, to transition users of epoch 0 to epoch 1, you would update your project
file to include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    epoch: 1*

Once this version is released, users from epoch 0 will be automatically migrated to
epoch 1 when they refresh. After the transition, they will upgrade to any new releases
in epoch 1, just as a new user would.
