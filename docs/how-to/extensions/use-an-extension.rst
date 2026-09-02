.. _how-to-use-an-extension:

Use an extension
================

To use an extension in an app, list it in the app's ``extensions`` key in the snap's
project file. Here's an example of an app using the KDE neon 6 extension:

.. code:: yaml

    apps:
      kcalc:
        command: usr/bin/kcalc
        extensions:
          - kde-neon-6
