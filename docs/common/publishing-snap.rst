Publishing the snap
-------------------

To share a snap you need to publish it in the `Snap Store`_. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You'll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   snapcraft login

Reserve a name for the snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   snapcraft register mypythonsnap

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload the snap
~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   snapcraft upload --release=edge mypythonsnap_*.snap

If you're happy with the result, you can commit the :file:`snapcraft.yaml` to a GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.
