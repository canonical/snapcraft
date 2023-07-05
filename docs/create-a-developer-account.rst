.. 6760.md

.. _create-a-developer-account:

Create a developer account
==========================

To upload your snap to the `Snap Store <https://snapcraft.io/store>`__ and access its management web interface, you will need a developer account. If you don’t already have one, creating one is easy.

.. _create-a-developer-account-details:

Ubuntu One setup
----------------

The account is registered on `Ubuntu One <https://login.ubuntu.com/>`__, a single sign-on service for Ubuntu and affiliated projects. Head over to https://snapcraft.io/account and select the “I don’t have an Ubuntu One account” option.

.. figure:: https://assets.ubuntu.com/v1/d7966a51-sso-01.png
   :alt: Ubuntu SSO Create Account


Fill out the form that appears. Your “full name” and “username” will be displayed next to your app in the Snap Store, so you should choose appropriate branding. Use your organisation’s name for both if you are publishing on their behalf.

You will then receive an email asking you to verify your account. Click the verification link in the email and complete the reCAPTCHA challenge that follows.

Finally, you will be asked to review the terms of the developer programme. If you wish to revisit these at any time you can find them here:

-  `Developer Terms and Conditions <https://www.ubuntu.com/legal/terms-and-policies/developer-terms-and-conditions>`__
-  `Privacy Notice <https://www.ubuntu.com/legal/dataprivacy/snap-store>`__

Once you have accepted the terms, your developer account is immediately ready to start publishing snaps. The details of your account can be reviewed at any time from the menu on the top right.


.. _create-a-developer-account-developer-id:

Developer account ID
--------------------

It’s useful to know your developer account id when building `Ubuntu Core devices <https://ubuntu.com/core/docs/system-user>`__, interacting with the Snap Store `review team <https://forum.snapcraft.io/c/store-requests/19>`__ and using the `snapd API <https://snapcraft.io/docs/snapd-rest-api>`__.

It can be retrieved with the :ref:`snapcraft <snapcraft-overview>` command by first logging in to your Ubuntu One account:

.. code:: bash

   $ snapcraft login
   Enter your Ubuntu One e-mail address and password.
   If you do not have an Ubuntu One account, you can create one at https://snapcraft.io/account
   Email: <your email address>
   Password:
   Second-factor auth: 960067

   Login successful.

If you’re rather used purely web-based authentication method, Snapcraft includes experimental functionality for a new web-based authentication flow. This allows you to complete the login process in a simple, secure manner using the browser, and extends the macaroon-based authentication outlined above:

.. code:: bash

   $ snapcraft login --experimental-login
   Opening an authorization web page in your browser.

   If it does not open, please open this URL:
   https://api.jujucharms.com/identity/login?did=c0cf2e16bc2244001945a6b3fe6d56c4e35a8401a3678ecff9fce89ef6cd2583

Snapcraft will forward the query to your default browser and open the login page, where you can authenticate your identity. This could be Ubuntu SSO, optional MFA, and any other methods that you would use. Once you complete the authentication, you will see *Login successful* output to the command line.

If you do not wish to use the experimental login feature anymore, you need to logout to clear your credentials. You can then return to the standard login process.

.. code:: bash

   $ snapcraft logout
   Credentials cleared.

In scenarios where the web-based access may be restricted, developers can export the credentials with the *export-login [file]* command, and then use them on other systems by passing on the *–with creds-file* option to snapcraft log

Following a successful login, the ``snapcraft whoami`` command reveals your *developer-id*:

.. code:: bash

   $ snapcraft whoami
   email:        <your email address>
   developer-id: xSfWKGdLoQBoQx88vIM1MpbFNMq53t1f

In the example output above, the developer-id is ``xSfWKGdLoQBoQx88vIM1MpbFNMq53t1f``.


.. _create-a-developer-account-ssh-keys:

Adding SSH keys to your account
-------------------------------

The contents of one or more `SSH public keys <https://help.ubuntu.com/community/SSH/OpenSSH/Keys>`__ can be added to, and associated with, your Ubuntu One account.

This is an essential step if you want to install `Ubuntu Core <https://ubuntu.com/core/docs>`__ because a registered private/public key pair is used to access the device using SSH after installation.

If you don’t already have an SSH key pair, or would like to use a new one, the following command will generate a new pair:

.. code:: bash

   mkdir -p ~/.ssh
   chmod 700 ~/.ssh
   cd ~/.ssh
   ssh-keygen -t rsa

You will be prompted for a filename and then for a passphrase. We recommend using a filename unique for this role, such as ``ucid_rsa``. Adding a passphrase secures against your private key being compromised, but it will need to be entered whenever the key is used.

.. code:: text

   Generating public/private rsa key pair.
   Enter file in which to save the key (/home/ubuntu/.ssh/id_rsa): ucid_rsa
   Enter passphrase (empty for no passphrase):
   Enter same passphrase again:
   Your identification has been saved in ucid_rsa
   Your public key has been saved in ucid_rsa.pub
   The key fingerprint is:
   SHA256:SCFVqXpDet/ZFKUxNYXkrJFpxz4n6QtI4S9KMgVZh14 ubuntu@2004-desktop
   The key's randomart image is:
   +---[RSA 3072]----+
   |     +*AC. o..   |
   |    .+.X=o=..E   |
   |    . B.-=. .    |
   | .   *o+ o*.     |
   |+ . . +oSo.      |
   |+o.. .  o        |
   | o.....  .       |
   |    .S.          |
   | ..o.            |
   +----[SHA256]-----+

The output is a 2048-bit RSA key pair which is secure enough for most use cases (you may optionally pass in the ``-b 4096`` flag to the ``ssh-keygen`` command, to create a larger 4096-bit key).

The contents of the ``<key name>.pub`` file (**not** the private key without an extension) now need to be pasted into the Public SSH Key field on https://login.ubuntu.com/ssh-keys, such as with *xclip*:

.. code:: bash

   $ cat ~/.ssh/ucid_rsa.pub | xclip
   sh-rsa AAAAB3N[...]ubuntu@2004-desktop

|Import new SSH key| Press the *Import SSH key* button to complete the import process.

The key should now be listed beneath *SSH Keys* at the top of the page which means the key has been recognised and is ready to use. Old and redundant keys should be removed with the ‘Delete selected keys’ button to avoid the potential security risk of someone using an old key.

.. |Import new SSH key| image:: https://assets.ubuntu.com/v1/611268cf-sso-02.png
