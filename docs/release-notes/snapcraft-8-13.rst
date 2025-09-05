.. _release-8.13:

Snapcraft 8.13 release notes
============================

.. add date here, once scheduled

Learn about the new features, changes, and fixes introduced in Snapcraft 8.13.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.13 brings the following features, integrations, and improvements.


Set component versions dynamically
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. include:: /reuse/bases-intro.rst

Similar to setting the top-level version, a component's version can now be set
dynamically with craftctl.

Additionally, the version for each component can be set by a unique part. This is
controlled with a new ``adopt-info`` key for each component.

For detailed guidance, see
:ref:`how-to-access-project-variables-across-parts-and-components`.
