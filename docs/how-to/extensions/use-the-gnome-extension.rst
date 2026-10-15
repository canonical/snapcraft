.. meta::
    :description: How to incorporate the GNOME extension into a snap.

.. _how-to-use-the-gnome-extension:

Use the GNOME extension
=======================

To use the :ref:`reference-gnome-extension` with an app, add it to the app's
``extensions`` key in the snap project file. For example:

.. code:: yaml

    apps:
      tali:
        extensions: [gnome]
        command: usr/bin/tali

For a comprehensive example of a snap project file that includes the extension, see
:ref:`how-to-craft-a-gtk4-app`.


Additional interfaces
---------------------

When you include this extension, a number of :ref:`plugs
<gnome-extension-included-plugs>` are automatically opened, so you won't need to declare these if needed.

On core26, the extension doesn't add the ``gsettings`` plug automatically.
Whether you should add it depends on :ref:`how your app uses GSettings
<how-to-use-the-gnome-extension-gsettings>`.

For a comprehensive look, you can preview all the keys the extension will add to your
project file. At the root of your project, run:

.. code-block:: bash

    snapcraft expand-extensions

Expanding the extensions prints your project file to the terminal exactly as it would be
transformed by the preprocessor immediately prior to build. The output reveals all the
keys and their default values.


.. _how-to-use-the-gnome-extension-gsettings:

GSettings
---------

For a new core26 snap, add the ``gsettings`` plug only if the app needs to read
or modify shared host settings. This access should be limited to settings
managers, whose purpose is to manage those settings. Add ``gsettings`` to the
app's ``plugs`` list in your project file, keeping any existing plugs.

The plug isn't required to store an app's own configuration with GSettings.
Without the plug, the app still uses GSettings to store configuration in a
private database, as with ``GSETTINGS_BACKEND=keyfile``. This behavior doesn't
provide access to shared host settings.


Library dependencies
--------------------

On core26, use the libraries supplied by the GNOME content snaps rather than
bundling alternative versions. The extension automatically removes bundled
shared libraries already supplied by the content snaps during prime.

The :ref:`library cleanup reference <reference-gnome-extension-library-cleanup>`
describes the constraints on shipping alternative versions and how to request
an update to an included library.

If you only need a newer version of a library already included in the GNOME
content snap, contact its maintainers in the `GNOME SDK repository
<https://github.com/ubuntu/gnome-sdk>`__ to request an update.
