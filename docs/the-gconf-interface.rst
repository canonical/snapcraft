.. 26499.md

.. _the-gconf-interface:

The gconf interface
===================

The ``gconf`` interface allows access to any item from the legacy `GConf configuration system <https://gitlab.gnome.org/Archive/gconf>`__ for the current user, typically used by old GNOME desktop libraries and applications.

This interface needs to be manually connected because *gconf* is a global database for GNOME desktop and application settings and offers no application isolation.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

Modern applications should use dconf/gsettings instead and this interface is provided for old codebases that cannot be migrated.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/gconf_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/gconf.go
