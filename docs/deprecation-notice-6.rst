.. 8406.md

.. _deprecation-notice-6:

Deprecation notice: 6
=====================

**Use of the snap command with a directory has been deprecated in favour of the pack command.**

*introduced in snapcraft 2.35*

The ``snap`` command originally served two purposes:

1. ``snapcraft snap`` walked through the entire lifecycle for each part of the project (pull, build, stage, and prime) before finally creating the snap, and
2. ``snapcraft snap <directory>`` simply created a snap of the provided directory (without involving the lifecycle at all).

This ended up being confusing from the point of view of someone working on an actual snapcraft project.

The new command ``pack`` is much more explicit in that snapcraft is just focusing on assembling the final snap from a valid layout for a given directory.

The new way to create a snap from a directory has the following syntax:

.. code:: bash

   snapcraft pack <directory>

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
