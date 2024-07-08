Bases
=====

.. include:: /reuse/bases-intro.rst

``base`` and ``build-base``
---------------------------

If ``build-base`` is defined , then the ``build-base`` will determine the
environment where the snap is built.

If ``build-base`` is not defined, the ``base`` will determine the
environment where the snap is built.

For example, ``base: core24`` will build a snap in a Ubuntu 24.04 environment
with the ``core24`` snap installed. Stage packages will be installed from the
24.04 repository.

``build-base: devel`` will build a snap using the upcoming Ubuntu release in
development. This is defined as the Ubuntu image with the ``devel`` alias in
the `Ubuntu buildd image server`_.

Base snaps
----------

A base snap is a special kind of snap that provides a run-time environment with
a cardinal set of libraries that are common to most applications. They’re
transparent to users, but they need to be considered and specified when building
a snap.

Mounting
--------

The base snap mounts itself as the root filesystem within a snap's runtime
environment. When an application runs, the base's library paths are searched
directly after the paths for that snap.

Choosing a base
---------------

``core24`` is the recommended base for most snaps. Much like choosing a
distribution base for a project or server, the best base for an application is
dependent on an application’s requirements and which plugins or extensions a
base supports. If there are specific dependencies that cannot be easily met
then the next newest base ``core22`` is a valid and supported alternative.

Snapcraft `extensions`_ enable bundling a set of common requirements into a
snap, such as for running KDE Plasma or GNOME applications. Extensions support
specific bases. See `supported extensions`_ for a list of which extensions
support which bases.

.. _base-snap-explanation:

Building a base snap
--------------------

While uncommon, developers can build their own base snap. They are responsible
for maintenance and updates, in particular:

* bases need to be built from stable packages
* ABI compatibility cannot broken (ie. never replace symbols or libraries, and
  be strict)
* security updates must be proactive

Base snaps can be either bootable or non-bootable. The former needs to include
``systemd`` while the latter can be leaner.


.. _`Ubuntu buildd image server`: https://cloud-images.ubuntu.com/buildd/daily/
.. _`extensions`: https://snapcraft.io/docs/snapcraft-extensions
.. _`supported extensions`: https://snapcraft.io/docs/supported-extensions
