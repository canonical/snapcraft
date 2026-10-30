.. meta::
    :description: Learn about the new features, changes, and fixes introduced in Snapcraft 9.3.

.. _release-9.3:

Snapcraft 9.3 release notes
===========================

Learn about the new features, changes, and fixes introduced in Snapcraft 9.3.


Requirements and compatibility
------------------------------

See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


Minor features
--------------

Snapcraft 9.3 brings the following minor changes.

Close a channel for specific architectures
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``close`` command now has an ``--arch`` option to close a channel for only the given
architectures. Snapcraft closes the channel and then releases the revisions of the other
architectures to it again. See :ref:`how-to-manage-revisions-and-releases` for details.
