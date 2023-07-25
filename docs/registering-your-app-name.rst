.. 6793.md

.. _registering-your-app-name:

Registering your app name
=========================

Now that you’ve :ref:`created a developer account <create-a-developer-account>` on the Snap Store and :ref:`built your snap <building-the-snap>` you can register a name for your app.

What name to choose
-------------------

Pick the name that most represents your app and use lowercase. As an example, a snap of the Firefox web browser should be called “firefox”. Do not prefix or suffix the name, for instance with your username or “-snap.”

Snap names are globally unique and cannot be changed. For example, only one snap can be named “firefox” and it can never be renamed. Ultimately, **each name should be owned and published by members of the relevant project.** For example, the snap named “firefox” should be owned and published by the developers of Firefox: the Mozilla project.

However, if you are not associated with the project but want to help them create a snap, we welcome you to :ref:`join snapcrafters <join-snapcrafters>`, create the snap yourself, register the name and hand off to upstream projects when asked.

How to register the name
------------------------

Make sure that the name you wish to register is the same as the ``name`` field in the :file:`snapcraft.yaml` file of your snap. You’ll need to rebuild your snap after you changed the name, a quick process when only the snap name has changed.

If you are working with an Electron app, you will not have a snapcraft.yaml file. If your snap name differs from the ``name`` property in your package.json file, set the ``executableName`` property, `under the top-level linux key <https://www.electron.build/configuration/linux>`__, to your snap name.

You also have a choice over whether a snap is *public* or *private*:

- **Public**: the snap will appear in local ``snap find`` searches, in the Snap Store, and other application installers that access the store
- **Private**: the snap is hidden from search results, and can only be installed from your account and from accounts linked to your account

To change a snap’s visibility after registering, see :ref:`release-management`.

A name can be registered from either the web UI or with the *snapcraft* command:

Register with the web UI
~~~~~~~~~~~~~~~~~~~~~~~~

To register a name with the web UI, log into your `developer account <https://snapcraft.io/account>`__ and click “Add snap” at the top of the page. You will be taken to a page where you can enter your snap’s name to register.

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/b/b1f74bd8422bf8196cd3b334eafd173350ad432d.png
   :alt: Screenshot_20200828_114733|613x361


If the name you want is already in use and you believe you are the rightful owner, you can register a dispute. Follow the on-screen instructions to be guided through this process. Your dispute will be reviewed by a member of the Snap Store team and a decision communicated by email.

   ⓘ The store team has pre-registered a set of common application names. These will be transferred to the relevant project upon request, using the *dispute* process outlined above.

Two further options enable you to register the snap as either *public* or *private*.

Register from the terminal
~~~~~~~~~~~~~~~~~~~~~~~~~~

To register a name with the web UI, log into your `developer account <https://snapcraft.io/account>`__ with the *snapcraft login* command:

.. code:: bash

   snapcraft login
   Enter your Ubuntu One e-mail address and password.
   If you do not have an Ubuntu One account, you can create one at https://snapcraft.io/account
   Email: <email address>
   Password:  <password>
   Second-factor auth: <one time password, if enabled>

   Login successful.

You can now use the *snapcraft register* command to register your new snap’s name:

.. code:: bash

   $ snapcraft register <snap name>

   We always want to ensure that users get the software they expect
   for a particular name.

   If needed, we will rename snaps to ensure that a particular name
   reflects the software most widely expected by our community.

   For example, most people would expect 'thunderbird' to be published by
   Mozilla. They would also expect to be able to get other snaps of
   Thunderbird as '$username-thunderbird'.

   Would you say that MOST users will expect '<snap name>' to come from
   you, and be the software you intend to publish there? [y/N]: y

As shown in the above output, you need to agree that most users will expect your snap name to represent the snap you’re wanting to publish.

By default, a newly registered snap is published as a *public* snap. It can be registered as *private* with the additional \`–private- argument:

.. code:: bash

   $  snapcraft register --private <snap name>

Creating an unofficial fork of a snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is a single exception for having your username as a suffix in a snap name: an unofficial snap that has **no** chance of being handed over to the official project.

This should be done with extreme caution because we don’t want to end up in a situation where, for example, “firefox-john” gets super popular and the developer decides they want to call it “firefox”. This is not possible because snaps cannot be renamed. So it will be “firefox-john” for ever, or they upload a second “firefox” snap and you end up orphaning the “firefox-john” snap and those users will not get updates.

Next steps
~~~~~~~~~~

See :ref:`Releasing your snap <releasing-your-app>` to learn how to upload your app to the Snap Store.
