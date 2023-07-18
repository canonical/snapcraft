.. 6795.md

.. _releasing-your-app:

Releasing your app
==================

After :ref:`creating a snap <creating-a-snap>`, you should upload it to the `Snap Store <https://snapcraft.io/store>`__, from where it can reach a potential audience of millions, or remain *private* if registered as such.

You will need the following:

- a free Snapcraft :ref:`developer account <create-a-developer-account>` account
- your own built and tested snap working with strict or classic :ref:`confinement <snap-confinement>`.

   ⓘ If your snap requires :ref:`classic confinement <classic-confinement>`, your snap will need manual approval before being released. See :ref:`Classic confinement review process <process-for-reviewing-classic-confinement-snaps>` for further details.

Publishing process
------------------

To get started, first :ref:`register a name <registering-your-app-name>` for your snap in the Snap Store.

Return to the terminal and the location of your ``.snap`` file. You now need to authenticate the *snapcraft* command using your Snapcraft developer account credentials. This can be accomplished with the following:

.. code:: bash

   snapcraft login

Next, upload the snap and release it into the stable :term:`channel`:

.. code:: bash

   snapcraft upload --release=stable mysnap_latest_amd64.snap

If no errors are detected in the automated review of your upload, your app will be immediately available for installation.

.. note::
          ⓘ If errors are detected, the snapcraft command will give a brief summary and guidance on how to correct each.

You can now test-install your snap from the Snap Store, ideally from a different testing environment to the one used to build your snap:

.. code:: bash

   sudo snap install mysnap

Congratulations, your snap has now been released and is available on the Snap Store!

See `Store listing and branding <https://snapcraft.io/docs/store-listing-and-branding>`__ for help with making the most of a snap’s store entry, and `Release management <https://snapcraft.io/docs/release-management>`__ for controlling which revisions appear on which channels, and to switch a snap between *Public* and *Private* visibility and access.

If you want to publish a snap temporarily, to address a fix or test a new feature, see :ref:`Publish to a branch <publish-to-a-branch>`.
