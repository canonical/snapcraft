.. _explanation-snap-configurations:

Snap configurations
===================

Snaps can provide `configurable options
<https://snapcraft.io/docs/configuration-in-snaps>`_ so that the user can change
behavior at runtime.


Implementation in a snap
------------------------

The standard setup is split across four scripts:

- A **configure hook script** makes calls to change the options on installation and on
  user input.
- An optional **default-configure hook script** makes calls to change the options when
  the snap is first installed, before services are started.
- A **wrapper script** launches the app with the necessary environment variables.
- A shared **management script** provides common functions for setting values and
  checking their validity.

In a typical setup, configuration options are mapped to environment variables within the
snap's runtime. The environment variables are typically already established in one or
more of the apps' namespaces. When a variable changes because the user sets an option,
the app picks up the change.

Every time the user changes the configuration, the configure hook script executes.
Through functions in the management script, this hook will typically validate the
configuration and, for example, update environment variables or write to the necessary
configuration files.


Interpreting options
--------------------

Internally, snaps view and change their configuration with `snapctl
<https://snapcraft.io/docs/how-to-guides/snap-development/use-snapctl/>`__ and its
``get``, ``set``, and ``unset`` arguments.

The snapctl command works anywhere within the snap context, during execution of your
apps and services, and in all the snap's hooks.

Configuration options are not defined when a snap is created because any valid option
name is accepted. Instead, any set values need to be interpreted and converted into an
action by the snap developer.

A snap developer is free to implement this process however they prefer, but it's most
commonly accomplished with a purpose-built script or function for each option, as
defined by a snap's project file and its associated scripts and :ref:`hooks
<reference-hooks>`.

If a hook alters the configuration and exits with a non-zero status code, the changes
won't apply. This is because the hook context is transactional -- either every change is
applied, or none are.

Permitted values should then be documented in the snap description so that users know
which values are supported.


Default values
--------------

The snap daemon has no concept of *default values* for configuration options. Actions
for these values need to be implemented by the snap developer using the :ref:`configure
hook <how-to-add-a-snap-configuration-configure-hook>`.

When a user resets a configuration option with ``snap unset``, or installs a snap, the
configure hook is run and the snap developer can therefore use this hook to check when
these values are unset and, if so, use ``snapctl set`` to restore that option to its
default value.

Setting these values explicitly is preferred over using implicit defaults in the snap,
because this way, users can easily discover which configuration options your snap
supports.

On Ubuntu Core, a device's `gadget snap <https://snapcraft.io/docs/the-gadget-snap>`_
can share default configuration options with the snaps listed in its ``gadget.yaml``
file. These options are shared when a snap is first installed using either the
:ref:`default-configure hook <how-to-add-a-snap-configuration-default-configure-hook>`,
which is run before services are started, or with the :ref:`configure hook
<how-to-add-a-snap-configuration-configure-hook>`, which runs after services are
started.


Nested values
-------------

You can group configuration options using a dotted path:

.. code-block:: bash

    snapctl set my-snap server.protocol=tcp server.port=4242

Each configuration option can be retrieved by using the same dotted path, or you can
retrieve the entire collection as a JSON document by specifying their common key:

.. terminal::
    :input: snapctl get server
    :user: crafter
    :host: host

    {
        "protocol": "tcp",
        "port": "4242"
    }
