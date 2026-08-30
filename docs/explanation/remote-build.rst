.. _explanation-remote-build:

Remote build
============

Remote build is a feature in Snapcraft that offloads the build process to
`Launchpad`_'s `build farm`_ and enables developers to build snaps for
different architectures.

Architectures supported by Launchpad can be found
:ref:`here <supported-architectures>`.


Public and private projects
---------------------------

By default, prospective snaps are publicly uploaded to `Launchpad`_.

Developers are reminded of this by confirming that their project will be
publicly available when starting a remote build. This prompt can be
automatically agreed to by passing ``--launchpad-accept-public-upload``.

Private projects can still be built using the remote builder. This requires
the user to create a private `Launchpad project`_ and pass the project with the
``--project <project-name>`` command line argument. An SSH key must be
registered in Launchpad because source code is uploaded using SSH.


Git repository
--------------

Projects must be in the top level of a git repository because snapcraft uses
a git-based workflow to upload projects to Launchpad.

Shallowly cloned repositories are not supported (e.g. ``git clone --depth
1``)
because git does not support pushing shallow clones.


Command line options
--------------------

``--build-for``
~~~~~~~~~~~~~~~

**Type**: Comma-separated list of strings

**Default**: The architectures specified in your project file or your host architecture

.. note::
   ``--build-for`` behaves differently for ``remote-build`` than it does for
   :ref:`lifecycle commands<reference-lifecycle-commands>`.

Remote builds are useful for building snaps on different architectures. Due
to this, the semantics for the ``--build-for`` argument is more complex than
when building a snap locally.

The argument operates in one of two different ways depending on the presence
of a ``platforms`` or ``architectures`` key in the project file.

The first mode of operation is when the ``platforms`` or ``architectures``
key is present in the project file. In this scenario, ``--build-for`` operates
similar to how it does for lifecycle commands. The difference from its usage in
lifecycle commands is that ``--build-for`` may be a comma-separated list, which
allows multiple snaps to be built. For more information about build plans and
filtering, see :ref:`Build plans <build-plans>`.

The second mode of operation is when there isn't a ``platforms`` or
``architectures`` key in the project file. In this scenario, ``--build-for``
defines the architectures to build for.


``--launchpad-accept-public-upload``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Bypasses the prompt that confirms whether you want to upload data to the public. It's
not necessary to use this flag if you used ``--project`` to specify a private project.


``--project``
~~~~~~~~~~~~~

**Type**: String

Explicitly specify a project to upload to.


``--launchpad-timeout``
~~~~~~~~~~~~~~~~~~~~~~~

**Type**: Integer

**Default**: 0

Time, in seconds, to wait for Launchpad to complete a build. A time of 0 seconds will
wait indefinitely.


``--recover``
~~~~~~~~~~~~~

Attempt to recover previously interrupted builds.


Project platforms and architectures
-----------------------------------

If the project metadata contains a ``platforms`` or ``architectures`` entry,
Snapcraft will request a build for each unique ``build-for`` architecture.

.. note::

    Launchpad does not support building multiple snaps on the same
    ``build-on`` architecture (`#4995`_).

If the project metadata does not contain a ``platforms`` or ``architectures``
entry and ``--build-for`` is not provided, Snapcraft will request a build on,
and for, the host's architecture.

.. _`Launchpad account`: https://launchpad.net/+login
.. _`Launchpad project`: https://launchpad.net/projects/+new
.. _`Launchpad`: https://launchpad.net/
.. _`build farm`: https://launchpad.net/builders
.. _`#4995`: https://github.com/canonical/snapcraft/issues/4995
