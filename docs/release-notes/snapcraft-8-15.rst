.. _release-8.15:

Snapcraft 8.15 release notes
============================

14 April 2026

Learn about the new features, changes, and fixes introduced in Snapcraft 8.15.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.

What's new
----------

Snapcraft 8.15 brings the following features, integrations, and improvements.

Support for bootstrapping stable base snaps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To allow bootstrapping new bases, base snaps can now be built with ``grade: stable``,
even when using the ``devel`` build base.


Minor features
--------------

Snapcraft 8.15 brings the following minor changes.

Improved support for packages in the Colcon plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`reference-colcon-plugin` now allows you to stage packages from ROS
content sharing snaps that have no upstream release.

Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

:literalref:`@Guillaumebeuzeboc <https://github.com/Guillaumebeuzeboc>`,
:literalref:`@medubelko <https://github.com/medubelko>`,
and :literalref:`@mr-cal <https://github.com/mr-cal>`.
