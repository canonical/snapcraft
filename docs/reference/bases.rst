Bases
=====

A base snap is a special kind of snap that provides a run-time environment with
a cardinal set of libraries that are common to most applications. They’re
transparent to users, but they need to be considered and specified when building
a snap.

Supported base snaps
--------------------

There are currently six supported bases:

* `core24`_: the newest base, built from `Ubuntu 24.04 LTS`_.
* `core22`_: built from `Ubuntu 22.04 LTS`_.
* `core20`_: built from `Ubuntu 20.04 LTS`_.
* `core18`_: the previous standard base for snap building, based on `Ubuntu
  18.04 LTS`_.
* `core`_: based on `Ubuntu 16.04 ESM`_, not to be confused with core16 (see
  below).
* `bare`_: an empty base that’s useful with fully statically linked snaps and
  when testing.

Older releases of core were occasionally referred to as *core 16*, but ``core``
and ``core16`` are now two distinct packages.

In most Ubuntu bases (except ``core``), snapd and its associated tools are
provided by their relevant snaps.

.. warning::

   `core16`_ should no longer be used. With no current stable release, its beta
   and candidate releases are classed as experimental, and packages previously
   using it should be moved to a more recent base.

Defining a base
---------------

Bases are defined by adding the base keyword to a snap’s `snapcraft.yaml`_
followed by the base name.

For example, to specify core20, use the following:

.. code-block:: yaml

   base: core20

To specify core22, use the following:

.. code-block:: yaml

   base: core22

Deprecated base snaps
---------------------

The latest releases of Snapcraft do not support older bases. Prior major
Snapcraft releases are still supported and can be installed from Snapcraft’s
`tracks`_.

``core18``
^^^^^^^^^^

To build ``core18`` snaps, install snapcraft 7 from the *7.x* track:

.. code-block:: shell

   $ snap install snapcraft --channel 7.x

``core``
^^^^^^^^

To build ``core`` snaps, install snapcraft 4 from the *4.x* track:

.. code-block:: shell

   $ snap install snapcraft --channel 4.x

For snaps using ``core``, we highly recommend reading `Snapcraft and ESM`_ for
essential support details.

The base snap mounts itself as the root filesystem within your snap such that
when your application runs, the base’s library paths are searched directly
after the paths for your specific snap.

.. warning::

   Compatibility mode and ESM Support

   When building a snap with no specified base, `Snapcraft`_ will operate in
   compatibility mode. This is essentially a prior (2.43-era) version of
   Snapcraft and, consequently, snapcraft will lose the functionality of newer
   releases. See `Snapcraft 3`_ release notes for details.

   This compatibility mode is no longer supported starting in Snapcraft 5.0.
   Snapcraft 4 can be installed from the 4.x track on the Snap Store (``snap
   install snapcraft --channel 4.x``). See `Snapcraft and ESM`_ for essential
   support details.

Choosing a base
---------------

``core22`` is the currently recommended base for the majority of snaps. But
much like choosing a distribution base for a project or server, the best base
for an application is dependent on an application’s requirements and which
plugins or extensions a base supports. If there are specific dependencies that
cannot be easily met with ``core22`` then ``core20`` is a valid and supported
alternative.

Snapcraft `extensions`_ are a great way to easily bundle a set of common
requirements into a snap, such as for running KDE Plasma or GNOME applications,
but you need to select a base that’s supported by the extension you require.
See `supported extensions`_ for a list of which extensions support which bases.

Base support was added with the release of `Snapcraft 3`_. As noted above,
snaps created before this, and snaps not using the base: keyword, can still be
built but they cannot use specific new features. Instead, snaps built without
bases inherit attributes from their respective build environments.

Snaps that don’t use bases can often migrate to one without too much
difficulty. See `upgrading snapcraft`_ for more details on potential
differences.

Building a base snap
--------------------

While it is possible to build your own base snap, its publisher needs to take
responsibility for its maintenance and updates. In particular:

* bases need to be built from stable packages
* ABI compatibility cannot broken (ie. never replace symbols or libraries, and
  be strict)
* security updates must be pro-active

Base snaps can be either bootable or non-bootable. The former needs to include
systemd while the latter can be leaner.

build-base
----------

The ``base`` keyword on its own does not take into account the creation of
bases. Instead, with older versions of snapcraft, the ``name`` keyword was
arbitrarily used to determine the build environment:

.. code-block:: yaml

   name: core18
   type: base
   # base: is not set elsewhere

The above example uses ``name`` to specify the creation of an Ubuntu 18.04
(``core18``) based build environment.

But the above fails if a base has yet to be bootstrapped, or is otherwise
unavailable. For example, the following will currently generate a ``launch
failed: Unable to find an image matching “futurecore”`` error:

.. code-block:: yaml

   name: futurecore
   type: base
   # base: is not set elsewhere

In snapcraft 7 and newer, a ``build-base`` keyword can be used to explicitly
define the base to use for the build environment where the base has not yet
been bootstrapped.

To solve the above issue, for example, use the following:

.. code-block:: yaml

   name: futurecore
   type: base
   build-base: core24
   # base: is not set elsewhere


.. _`Snapcraft 3`: https://snapcraft.io/docs/release-notes-snapcraft-3-0
.. _`Snapcraft and ESM`: https://snapcraft.io/docs/snapcraft-esm
.. _`Ubuntu 16.04 ESM`: https://releases.ubuntu.com/16.04/
.. _`Ubuntu 18.04 LTS`: https://releases.ubuntu.com/18.04/
.. _`Ubuntu 20.04 LTS`: https://releases.ubuntu.com/20.04/
.. _`Ubuntu 22.04 LTS`: https://releases.ubuntu.com/22.04/
.. _`Ubuntu 24.04 LTS`: https://releases.ubuntu.com/24.04/
.. _`bare`: https://snapcraft.io/bare
.. _`core16`: https://snapcraft.io/core16
.. _`core18`: https://snapcraft.io/core18
.. _`core20`: https://snapcraft.io/core20
.. _`core22`: https://snapcraft.io/core22
.. _`core24`: https://snapcraft.io/core24
.. _`core`: https://snapcraft.io/core
.. _`extensions`: https://snapcraft.io/docs/snapcraft-extensions
.. _`snapcraft.yaml`: https://snapcraft.io/docs/snapcraft-schema
.. _`supported extensions`: https://snapcraft.io/docs/supported-extensions
.. _`tracks`: https://snapcraft.io/docs/channels#heading--tracks
.. _`upgrading snapcraft`: https://snapcraft.io/docs/upgrading-snapcraft
