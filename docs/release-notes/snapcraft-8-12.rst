.. _release-8.12:

Snapcraft 8.12 release notes
============================

.. add date here, once scheduled

Learn about the new features, changes, and fixes introduced in Snapcraft 8.12.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.12 brings the following features, integrations, and improvements.


Minor features
--------------

Remote builds: ``--build-for`` and shorthand platforms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, the :ref:`remote builder <explanation-remote-build>` couldn't accept the
``--build-for`` argument when building core24 snaps with a shorthand :ref:`platforms
<reference_architectures>` definition.

Due to upstream improvements in Launchpad, this restriction has been lifted in
Snapcraft. Now, ``--build-for`` can be used for any core24 project.

