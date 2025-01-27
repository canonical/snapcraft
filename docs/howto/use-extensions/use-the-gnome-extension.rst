.. _use-the-gnome-extension:

Use the Gnome extension
=======================

To use the :ref:`gnome-extension` with an app, add it to the app's ``extensions``
key in the snap recipe. For example:

.. code:: yaml

    apps:
      tali:
        extensions: [gnome]
        command: usr/bin/tali

For a comprehensive example of snap recipe that includes the extension, see
:ref:`example-gtk4-app`.


Additional interfaces
---------------------

When you include this extension, a number of :ref:`plugs
<gnome-extension-included-plugs>` are automatically opened, so you won't need to declare these if needed.

For help with other plugs, see `Adding interfaces <https://snapcraft.io/docs/snapcraft-interfaces>`_.
