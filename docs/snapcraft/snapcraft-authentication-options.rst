.. 30473.md

.. _snapcraft-authentication-options:

Snapcraft authentication options
================================

Snapcraft’s login credentials can be exported and subsequently used on a system where login is not possible or desired, such as on a system that’s offline. A system keychain can also be used when a system is running without a connected display, as outlined below:

-  `Export snapcraft’s login credentials <snapcraft-authentication-export_>`__
-  `Using exported snapcraft credentials <snapcraft-authentication-using_>`__
-  `Verifying accepted credentials <snapcraft-authentication-verify_>`__
-  `Using a keyring on a headless Linux system) <snapcraft-authentication-keyring_>`__


.. _snapcraft-authentication-export:

Export snapcraft’s login credentials
------------------------------------

To export snapcraft’s login credentials, use the ``export-login`` command with the name of a file to store the credentials. On any system where Snapcraft is supported, run:

.. code:: bash

   snapcraft export-login <credentials-filename>

You will be asked for your email, password and second-factor authentication.

The format of exported credentials differ between Snapcraft versions 6 and 7+. Snapcraft 6 exports a decrypted macaroon, which should not be shared, while later releases output a raw block of base64 encoded text that can only be used with Snapcraft 7.0 or greater.

.. note::


          If you encounter *Cannot parse config* errors while processing authentication credentials, old credentials are likely stored the *snapcraft* configuration file which needs to be moved or deleted (``$HOME/.config/snapcraft/snapcraft.cfg``).


.. _snapcraft-authentication-using:

Using exported snapcraft credentials
------------------------------------

Previously exported credentials can be used to authenticate Snapcraft with an environment variable, or on older versions of Snapcraft, by using ``login --with`` and an external file.

SNAPCRAFT_STORE_CREDENTIALS environment variable
------------------------------------------------

On the system you wish to use previously exported credentials, the contents of the credentials file needs to be placed into an environment variable called ``SNAPCRAFT_STORE_CREDENTIALS``. This can be accomplished in many ways, but the following is a good solution:

.. code:: bash

   export SNAPCRAFT_STORE_CREDENTIALS=$(cat <credentials-filename>)

snapcaft login –with (Snapcraft 6.x only)
-----------------------------------------

In addition to the above, the ``snapcraft login`` command accepts an additional ``--with`` argument to reference a login credentials file.

.. code:: bash

   snapcraft login --with <credentials-filename>

.. note::


          The ``login --with`` argument is not supported in Snapcraft 7 and is currently included to help users migrate from the old authentication method to the new. The accompanying credentials must also be generated with Snapcraft 6.x.


.. _snapcraft-authentication-verify:

Verifying accepted credentials
------------------------------

Use ``snapcraft whoami`` to verify login credentials are working:

.. code:: bash

   $ snapcraft whoami
   email: <account-email>
   username: <account-name>
   id: <account-id>
   permissions: package_access, package_manage, package_metrics,
   package_push, package_register, package_release, package_update
   channels: no restrictions
   expires: 2023-06-15T14:49:49.000Z


.. _snapcraft-authentication-keyring:

Using a keyring on a headless Linux system
------------------------------------------

A Linux desktop will typically include an integrated keyring utility to store and retrieve passwords. This process can also be made to work on a headless system with no display connected or accessible desktop.

First, make sure ``gnome-keyring`` is installed:

.. code:: bash

   apt install gnome-keyring

Now start a dbus session:

.. code:: bash

   dbus-run-session -- sh

To unlock the keyring from the command-line, run the following. You will be asked to enter a passphrase, type ctrl+d when done:

.. code:: bash

   gnome-keyring-daemon --unlock

Now you can login as usual:

.. code:: bash

   snapcraft login
