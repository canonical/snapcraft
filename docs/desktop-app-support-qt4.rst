.. 11711.md

.. _desktop-app-support-qt4:

Desktop App support - Qt4
=========================

   This topic is a stub, feel free to expand it.

For Qt4 applications currently the recommended way to snap them is via the use of the ``desktop-qt4`` part provided by `the Snapcraft Desktop Helpers project <https://github.com/ubuntu/snapcraft-desktop-helpers>`__.

Merge the definition the ``desktop-qt4`` part from https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml into your Snapcraft recipe, with the ``source`` property changed to https://github.com/ubuntu/snapcraft-desktop-helpers.git

.. code:: yaml

   parts
     desktop-qt4:
       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git

           ...stripped...

..

   **NOTE:** In order to keep the integration up-to-date for the best user experience, you should check if the definition has been changed periodically.

Insert ``${SNAP}/bin/desktop-launch`` (or ``desktop-launch`` if the command search ``PATH``\ s allows) into the app’s command chain:

.. code:: yaml

   apps:
     qt4app:
       command: desktop-launch qt4app-launch qt4app

If you’re using the ``full`` adapter, insert the ``bin/desktop-launch`` launcher into the list of the ``command-chain`` key of the ``apps._app_name_`` property instead:

.. code:: yaml

   apps:
     qt4app:
       adapter: full
       command: bin/qt4app
       command-chain:
       - bin/desktop-launch
       - bin/qt4app-launch
