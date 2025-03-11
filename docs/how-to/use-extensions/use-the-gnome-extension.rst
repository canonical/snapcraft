.. _use-the-gnome-extension:

Use the GNOME extension
=======================

To use the :ref:`gnome-extension` with an app, add it to the app's ``extensions``
key in the snap project file. For example:

.. code:: yaml

    apps:
      tali:
        extensions: [gnome]
        command: usr/bin/tali

For a comprehensive example of a snap project file that includes the extension, see
:ref:`example-gtk4-app`.


Additional interfaces
---------------------

When you include this extension, a number of :ref:`plugs
<gnome-extension-included-plugs>` are automatically opened, so you won't need to declare these if needed.

For a comprehensive look, you can preview all the keys the extension will add to your
project file. At the root of your project, run:

.. code-block:: bash

    snapcraft expand-extensions

Expanding the extensions prints your project file to the terminal exactly as it would be
transformed by the preprocessor immediately prior to build. The output reveals all the
keys and their default values.

For help with other plugs, see `Adding interfaces
<https://snapcraft.io/docs/snapcraft-interfaces>`_.
