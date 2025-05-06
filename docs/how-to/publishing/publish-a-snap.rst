.. _how-to-publish-a-snap:

Publish a snap
==============

After :ref:`crafting a snap <tutorial-craft-a-snap>`, you should upload it to the Snap
Store or a private store so that others can use it.


Prepare to register
-------------------

First, register for a Snapcraft account and :ref:`log in to your account
<how-to-autheneticate-log-in>` in a terminal.

Then, :ref:`register your snap's name <how-to-register-a-snap>`.

In rare cases, you must also :ref:`receive prior approval
<how-to-enable-classic-confinement-request-snap-store>` to publish if your snap requires
classic confinement.


Publish a new snap
------------------

Once the prerequisites are met, you can publish the first stable release of your snap.

In the terminal, go to your built snap file.

Next, upload the first revision on the stable channel:

.. code-block:: bash

    snapcraft upload --release=stable <my-snap>.snap

After receiving the upload, the store performs an automated review of the snap file.
If no errors are found, the store makes the snap immediately available to users.

If the automated check finds any errors, Snapcraft will give a brief summary and
guidance on how to correct each. Correct all the errors, rebuild the snap, then upload
it again. Continue retrying until successful.

Your snap is now released and ready for people to try!


Test the availability locally
-----------------------------

It's a good idea to now test your snap from the store, ideally from a different testing
host than the one you used to initially build your snap:

.. code-block:: bash

    snap install <my-snap>
    <my-snap>

If you can successfully install and run the snap on a host, then your snap has completed
its first user test.


Next steps
----------

Now that your snap is published, you should begin :ref:`managing its revisions and
releases <how-to-manage-revisions-and-releases>`.
