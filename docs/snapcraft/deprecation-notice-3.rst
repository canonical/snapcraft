.. 8403.md

.. _deprecation-notice-3:

Deprecation notice: 3
=====================

**Assets in setup/gui should now be placed in snap/gui.**

*introduced in snapcraft 2.26*

Originally snapcraft had the idea of putting all fixed assets inside a ``setup`` directory, this was introduced with the capability of adding GUI assets such as desktop files and package icons.

With the introduction of the ``snap`` directory, the ``setup`` directory becomes superfluous and has been deprecated in favor of putting GUI assets in ``snap/gui`` instead of ``setup/gui``.

To move to the new schema, all you need to do is:

.. code:: bash

   $ mkdir -p snap
   $ mv setup/gui snap/

And if empty:

.. code:: bash

   $ rmdir setup

…and apply those changes to your VCS if you are using one.

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
