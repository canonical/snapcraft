.. _how-to-add-a-snap-configuration:

Add a snap configuration
========================

This guide describes how to set up :ref:`configuration options
<explanation-snap-configurations>` in your snap.


Review the requirements
-----------------------

In a standard setup for configuration options, you create four scripts:

- A **configure hook script** makes calls to change the options on installation and on
  user input.
- An optional **default-configure hook script** makes calls to change the options when
  the snap is first installed, before services are started.
- A **wrapper script** launches the app with the necessary environment variables.
- A shared **management script** provides common functions for setting values and
  checking their validity.

You must define a configure hook. Otherwise, users can't set any options.

Once the scripts are ready, you link them in the project file.


Copy the standard setup
-----------------------

The following example scripts are for a snap containing an app named ``example-server``.
They add a configuration option for specifying the server's port.

For a standard setup, recreate their implementation in your project.


.. _how-to-add-a-snap-configuration-configure-hook:

Add the configure hook script
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When the snap is installed, and when the user sets or unsets any option, the configure
hook is called.

The following example checks the validity of the port value and runs the custom
``set_http_port`` function provided by the management script before restarting the
server and making sure the changes have taken hold:

.. code-block:: bash
    :caption: src/hooks/bin/configure

    #!/bin/sh

    # Source the management script
    . "$SNAP/utilities/bin/management"

    handle_port_config()
    {
        http_port="$(http_port)"

        # Validate HTTP port
        if ! expr "$http_port" : '^[0-9]\+$' > /dev/null; then
                echo "\"$http_port\" is not a valid HTTP port" >&2
                return 1
        fi

        # Run function from management script
        set_http_port "$http_port"

        # Restart example-server to apply new config
        snapctl restart example-server
    }
    handle_port_config


.. _how-to-add-a-snap-configuration-default-configure-hook:

Add the default-configure hook script
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The default-configure hook optionally extends the configure hook and executes only on
snap installation and before services are started.

The following example retrieves a default configuration option from a gadget and either
writes it to a file, or writes a fallback value if the gadget option doesn't exist:

.. code-block:: bash
    :caption: src/hooks/bin/default-configure

    #!/bin/sh

    DEFAULT_GADGET_OPTION="123"

    gadget_option="$(snapctl get gadget_option)"
    if [ -z "$gadget_option" ]; then
      gadget_option="$DEFAULT_GADGET_OPTION"
    fi

    mkdir -m 0600 $SNAP_DATA/options
    echo "option: $gadget_option" > $SNAP_DATA/options/gadget


Add the wrapper script
~~~~~~~~~~~~~~~~~~~~~~

The wrapper script retrieves the current values of the options, and maps them to
environment variables which can be used as arguments when running ``example-server``.

.. code-block:: bash
    :caption: src/example-server/bin/wrapper

    #!/bin/sh

    # Source the management script
    . "$SNAP/utilities/bin/management"

    # Call the http_port function from the management script
    HTTP_PORT="$(http_port)"
    export HTTP_PORT

    "$SNAP/bin/example-server" -www "$HTTP_PORT"

.. admonition:: Further development

    Rather than setting individual environment variables for an executable, they could
    be written to a configuration file.


Add the management script
~~~~~~~~~~~~~~~~~~~~~~~~~

A separate script for management functions makes the functions accessible from both the
wrapper and the configure hook scripts.

The following example defines a default HTTP port and two functions:

- ``http_port`` requests the default port if nothing is yet set and returns the port
  value.
- ``set_http_port`` sets the port value.

Observe how the port itself is obtained from a call to snapctl. It acts as the
intermediary for all option values.

.. code-block:: bash
    :caption: src/utilities/bin/management

    #!/bin/sh

    DEFAULT_HTTP_PORT="80"

    http_port()
    {
        port="$(snapctl get ports.http)"
        if [ -z "$port" ]; then
            port="$DEFAULT_HTTP_PORT"
            set_http_port $port
        fi
        echo "$port"
    }

    set_http_port()
    {
        snapctl set ports.http="$1"
    }

.. admonition:: Further development

    This script could be expanded to manage the running process, and check whether the
    new port value is different from the old, saving the service from potentially
    restarting.


Source the scripts in the project file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To incorporate options, hooks, and scripts into an existing project file, the app's
command must be replaced with the wrapper script, and both the hook and management
scripts need to be brought into the snap from external ``src/hooks/bin`` and
``src/utilities/bin`` directories respectively:

.. code-block:: yaml
    :caption: snapcraft.yaml

    apps:
      example-server:
        command: src/example-server/bin/wrapper
        daemon: simple
        plugs: # ...
    # ...
    hooks:
        plugin: dump
        source: src/hooks/
        organize:
          bin/: snap/hooks/
    # ...
    scripts:
      plugin: dump
      source: src/utilities


Test the option
~~~~~~~~~~~~~~~

Build and install the snap.

Then, test getting and setting the port with:

.. terminal::
    :user: crafter
    :host: home
    :input: snap set example-server ports.http=8090

    :input: snap get domoticz-gm ports.http
    8090


Example live snap
-----------------

The `Nextcloud snap <https://snapcraft.io/nextcloud>`_ has a working example of a
configuration option. Its `setup
<https://github.com/nextcloud-snap/nextcloud-snap/blob/master/README.md#configuration>`__
configures the hostname, ports, and PHP memory limit:

.. code-block:: bash

    sudo snap set nextcloud ports.http=81
