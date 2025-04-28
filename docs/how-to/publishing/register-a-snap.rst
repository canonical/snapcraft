.. _how-to-register-a-snap:

Register a snap
===============

Before you can release your snap on the Snap Store, it must first be registered with
a well-formed name.


Log in to the Snap Store
------------------------

Before registering a snap, you must first log in to your developer account. If you don't
already have an account, create one `here <https://login.ubuntu.com/>`_.

To log in to your account from the terminal, run:

.. code-block:: bash

    snapcraft login

This will prompt you to enter your Ubuntu One e-mail, password, and, if applicable, your
two-factor authentication code.


Name your snap
--------------

When deciding on a name for your snap, it's important to follow the best naming
practices, as a snap's name is universally unique among all public and private snap
stores and can't be changed after registration.

The name you register should accurately represent your software, and must:

- Contain no more than 40 characters
- Consist of only lowercase letters, numbers, and hyphens
- Contain at least one letter
- Not start or end with a hyphen

Generally, a snap's name should also be free of prefixes or suffixes (for example,
``<username>-`` or ``-snap``). However, if you are registering an unofficial snap that
has no chance of being handed over to the official project, your username should be
appended to the app name. For example, if you were creating an unofficial fork of
Firefox, you would name it ``firefox-<username>``.

Ensure that the value of the ``name`` key in your snap's project file matches the name
you wish to register. You can also include the optional ``title`` key to define a
human-friendly name for use in in graphical frontends, such as the Snap Store and the
Ubuntu App Center.

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: my-new-snap
    title: My New Snap

If you need to make any changes to your project file, be sure to rebuild the snap before
registration.


Register your snap
------------------

Once you have logged in and decided on a name for your snap, you can register it by
running:

.. code-block:: bash

    snapcraft register <snap-name>

By default, a newly registered snap is published as a public snap. If you instead
wish to make it private, append the ``--private`` command option:

.. code-block:: bash

    snapcraft register --private <snap-name>

This will hide the snap from search results and ensure that it can only be downloaded
from your account or from accounts linked to your account.
