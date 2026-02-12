.. _how-to-authenticate:

Authenticate
============

To manage and publish snaps on a snap store, you must be logged in to a Snapcraft
developer account. File-based authentication is also available for deployments where
login isn't possible or desired, such as private networks.


.. _how-to-autheneticate-log-in:

Log in to your Snapcraft account
--------------------------------

To log in to your Snapcraft developer account from the terminal, run:

.. code-block:: bash

    snapcraft login

This will prompt you to enter your email, password, and, if applicable, your second
factor authentication code.


Export Snapcraft account credentials
------------------------------------

.. note::

    Credentials exported with Snapcraft 7 can only be used with Snapcraft 7 or
    higher.

To export Snapcraft account credentials, run the ``export-login`` command followed
by a name for the resulting credentials file:

.. code-block:: bash

    snapcraft export-login <credentials-filename>

Like ``snapcraft login``, this will prompt you to enter your email, password, and second
factor authentication code. Once entered, your account credentials will be exported to a
file in your current directory with the name you specified.

By default, the exported credentials will have access to all snaps, channels, and ACLs
associated with your account. To limit this access, you can append any of the following
options to the previous command:

.. list-table::
    :header-rows: 1

    * - Option
      - Argument
    * - ``--snaps``
      - Comma-separated list of allowed snaps
    * - ``--channels``
      - Comma-separated list of allowed channels
    * - ``--acls``
      - Comma-separated list of allowed ACLs
    * - ``--expires``
      - The date and time (in ISO 8601) the exported
        credentials will expire.


Authenticate with exported account credentials
----------------------------------------------

To authenticate with exported credentials, the contents of the credentials file must be
placed in the ``SNAPCRAFT_STORE_CREDENTIALS`` environment variable. This can be
accomplished by running:

.. code-block:: bash

    export SNAPCRAFT_STORE_CREDENTIALS=$(cat <credentials-filename>)

On older versions of Snapcraft, you can also authenticate with exported credentials by
running:

.. code-block:: bash

    snapcraft login --with <credentials-filename>


Verify exported account credentials
-----------------------------------

After authenticating with a file, you can verify that your account credentials are
working with:

.. code-block:: bash

    snapcraft whoami

This should output your account information as follows:

.. terminal::
    :input: snapcraft whoami

    email: <account-email>
    username: <account-name>
    id: <account-id>
    permissions: package_access, package_manage, package_metrics,
    package_push, package_register, package_release, package_update
    channels: no restrictions
    expires: 2026-03-17T14:29:45.000Z


Authenticate with a keyring
---------------------------

On systems where you wish to remain logged in, run ``snapcraft login``. Snapcraft will
attempt to use the system keyring. If no keyrings are installed or initialized,
Snapcraft will fall back to file-based credential storage. The file-based storage is
managed by Snapcraft. If you wish to import/export credentials, refer to the previous
sections.

Note that ``snapcraft login`` sometimes fails to unlock GNOME keyring when accessing a
Linux system with a desktop environment from a virtual console or SSH. GNOME keyring
will not present a CLI password prompt to unlock the keyring, causing Snapcraft to stall
and timeout.
