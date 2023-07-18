.. 11198.md

.. _base-snaps:

Base snaps
==========

A *base* snap is a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered and specified when building a snap.


.. _base-snaps-supported:

Supported base snaps
--------------------

There are currently five supported bases:

- `core22 <https://snapcraft.io/core22>`__: the newest base, built from `Ubuntu 22.04 LTS <https://releases.ubuntu.com/22.04/>`__.
- `core20 <https://snapcraft.io/core20>`__: built from `Ubuntu 20.04 LTS <https://releases.ubuntu.com/20.04/>`__.
- `core18 <https://snapcraft.io/core18>`__: the previous standard base for snap building, based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.
- `core <https://snapcraft.io/core>`__: based on `Ubuntu 16.04 ESM <http://releases.ubuntu.com/16.04/>`__, not to be confused with ``core16`` (see below).
- `bare <https://snapcraft.io/bare>`__: an empty base that’s useful with fully statically linked snaps and when testing.

Older releases of ``core`` were occasionally referred to as *core 16*, but ``core`` and ``core16`` are now two distinct packages.

In most Ubuntu bases (except ``core``), *snapd* and its associated tools are provided by their relevant snaps. `core16 <https://snapcraft.io/core16>`__ should no longer be used. With no current stable release, its beta and candidate releases are classed as experimental, and packages previously using it should be moved to a more recent base.

`core16 <https://snapcraft.io/core16>`__ development has been dropped. With no current stable release, its beta and candidate releases are classed as experimental. As with ``core18`` (and distinct from ``core``), it’s built without bundling *snapd* and its associated tools. In both ``core16``, ``core18`` and ``core20``, these are instead provided by their associated snaps.

Defining a base
---------------

Bases are defined by adding the ``base:`` keyword to a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` followed by the base name.

For example, to specify core 18, use the following:

.. code:: yaml

   base: core18

To specify *core22*, use the following

.. code:: yaml

   base: core22

Snapcraft no longer supports building snaps with the older *core* base. Snapcraft 4 needs to be used instead. Snapcraft 4 is still being supported and can be installed from Snapcraft’s *4.x* :term:`track`:

::

   $ snap install snapcraft --channel 4.x

For snaps using *core*, we highly recommend reading :ref:`Snapcraft and Extended Security Maintenance <snapcraft-and-extended-security-maintenance>` for essential support details.

The base snap mounts itself as the root filesystem within your snap such that when your application runs, the base’s library paths are searched directly after the paths for your specific snap.

.. note::
          Compatibility mode and ESM Support

          When building a snap with no specified base, :ref:`Snapcraft <snapcraft-overview>` will operate in compatibility mode. This is essentially a prior (2.43-era) version of Snapcraft and, consequently, *snapcraft* will lose the functionality of newer releases. See :ref:`release-notes-snapcraft-3-0-base-exceptions` for details.

          This compatibility mode is no longer supported in Snapcraft 5.0. Snapcraft 4 can be installed from the 4.x track on the Snap Store (``snap install snapcraft --channel 4.x``). See :ref:`Snapcraft and Extended Security Maintenance <snapcraft-and-extended-security-maintenance>` for essential support details.



Choosing a base
---------------

``core22`` is the currently recommended base for the majority of snaps. But much like choosing a distribution base for a project or server, the best base for an application is dependent on an application’s requirements and which plugins or extensions a base supports. If there are specific dependencies that cannot be easily met with ``core22`` then ``core20`` is a valid and supported alternative, as is the older ``core18``.

:ref:`Snapcraft extensions <snapcraft-extensions>` are used to bundle a set of common requirements into a snap, such as for running KDE Plasma or GNOME applications, but you need to select a base that’s supported by the extension you require. See :ref:`Supported extensions <supported-extensions>` for a list of which extensions support which bases.

Base support was added with the release of :ref:`Snapcraft 3 <release-notes-snapcraft-3-0>`. As noted above, snaps created before this, and snaps not using the ``base:`` keyword, can still be built but they cannot use :ref:`specific new features <release-notes-snapcraft-3-0-base-exceptions>`. Instead, snaps built without bases inherit attributes from their respective build environments.

Snaps that don’t use bases can often migrate to one without too much difficulty. See :ref:`Upgrading snapcraft <upgrading-snapcraft>` for more details on potential differences.

.. _building-a-base-snap:

Building a base snap
--------------------

While it is possible to build your own base snap, its publisher needs to take responsibility for its maintenance and updates. In particular:

-  bases need to be built from *stable* packages
-  ABI compatibility cannot broken (ie. never replace symbols or libraries, and be strict)
-  security updates must be pro-active

Base snaps can be either bootable or non-bootable. The former needs to include *systemd* while the latter can be leaner.


.. _base-snaps-base-snap:

build-base
----------

The ``base`` keyword on its own does not not take into account the *creation* of bases. Instead, with older versions of snapcraft, the ``name`` keyword was arbitrarily used to determine the build environment:

.. code:: yaml

   name: core18
   type: base
   # base: is not set elsewhere

The above example uses ``name`` to specify the creation of an Ubuntu 18.04 (core18) based build environment.

But the above fails if a base has yet to be bootstrapped, or is otherwise unavailable. For example, the following will currently generate a \`launch failed: Unable to find an image matching “futurecore” error:

.. code:: yaml

   name: futurecore
   type: base
   # base: is not set elsewhere

In cases like the above, where the base has not yet been bootstrapped, the ``build-base`` keyword should be used to explicitly define the base to use for the build environment.

To solve the above issue, for example, use the following:

.. code:: yaml

   name: futurecore
   type: base
   build-base: core18
   # base: is not set elsewhere
