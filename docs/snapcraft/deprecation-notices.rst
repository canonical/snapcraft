.. 8396.md

.. _deprecation-notices:

Deprecation notices
===================

This document contains a list of *snapcraft* deprecation notices and recommendations:

-  :ref:`DN1 <deprecation-notice-1>`: The ``snap`` keyword has been replaced by ``prime``
-  :ref:`DN2 <deprecation-notice-2>`: Custom plugins should now be placed in ``snap/plugins``
-  :ref:`DN3 <deprecation-notice-3>`: Assets in ``setup/gui`` should now be placed in ``snap/gui``
-  :ref:`DN4 <deprecation-notice-4>`: The ``history`` command has been renamed to ``list-revisions``
-  :ref:`DN5 <deprecation-notice-5>`: Aliases are now handled by the Snap Store, and shouldn’t be placed in the snap
-  :ref:`DN6 <deprecation-notice-6>`: Use of the ``snap`` command with a directory has been deprecated in favour of the ``pack`` command
-  :ref:`DN7 <deprecation-notice-7>`: The ``prepare`` keyword has been replaced by ``override-build`` (or :ref:`override-pull <override-build-steps>`)
-  :ref:`DN8 <deprecation-notice-8>`: The ``build`` keyword has been replaced by ``override-build``
-  :ref:`DN9 <deprecation-notice-9>`: The ``install`` keyword has been replaced by ``override-build``
-  :ref:`DN10 <deprecation-notice-10>`: The ``version-script`` keyword has been replaced by ``snapcraftctl set-version``
-  :ref:`DN11 <deprecation-notice-11>`: The ``push`` keywords have been replaced by ``upload`` equivalents
-  :ref:`DN12 <deprecation-notice-12>`: The ``registered`` and ``list-registered`` keywords has been replaced by ``list``
-  :ref:`DN13 <deprecation-notice-13>`: Support for legacy ``core`` projects will be removed in Snapcraft 5.0 (expected July 22, 2021)


.. toctree::
   :hidden:

   deprecation-notice-1
   deprecation-notice-2
   deprecation-notice-3
   deprecation-notice-4
   deprecation-notice-5
   deprecation-notice-6
   deprecation-notice-7
   deprecation-notice-8
   deprecation-notice-9
   deprecation-notice-10
   deprecation-notice-11
   deprecation-notice-12
   deprecation-notice-13
