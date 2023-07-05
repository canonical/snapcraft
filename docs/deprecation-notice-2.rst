.. 8398.md

.. _deprecation-notice-2:

Deprecation notice: 2
=====================

**Custom plugins should now be placed in snap/plugins.**

*introduced in snapcraft 2.26*

Originally snapcraft custom plugins were designed to be put in parts/plugins\` to avoid over-polluting the local directory with too many artifacts. A few problems manifested with this approach:

1. Many just deleted the ``parts`` directory where the plugins lived in to start over and were confused when their plugins suddenly weren’t found.
2. The need for new files in the layout.

As a result, a new schema was needed. The new design introduces the ``snap`` directory to hold all specific snapcraft assets, with this in mind a new location was set for custom plugins, and that is ``snap/plugins/<plugin-name>``.

To move to the new schema, for plugins all you need to do is:

.. code:: bash

   $ mkdir snap
   $ mv parts/plugins snap/

…and apply those changes to your VCS if you are using one.

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
