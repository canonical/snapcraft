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

Previously, a component could only set its version statically, when first declared.

Parts can now set a component versions dynamically. If a component points to a part with
the ``adopt-info`` key, the part can call craftctl to set the version.

For detailed guidance, see
:ref:`how-to-access-project-variables-across-parts-and-components`.
