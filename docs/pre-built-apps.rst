.. 6739.md

.. _pre-built-apps:

Pre-built applications
======================

Shipping a zip file or tarball of your application makes it easy for users to
download and start using it without the need for you to package it for each
Linux distribution.

However, this still requires that you provide instructions on installing the
application's dependencies, and it requires that you provide a method for
notifying users of available updates.

Snapcraft can be used to package and distribute pre-built applications in a
way that enables convenient installation by users while building on work you
have already done as part of your application's release process.


Getting started
---------------

Snaps are defined in a single :file:`snapcraft.yaml` file placed in a
:file:`snap` folder at the root of your project. This YAML file describes
the application, its dependencies and how it should be built.

The following example shows an entire :file:`snapcraft.yaml` file based on the
snap of an existing project, `Geekbench 4`_:

.. code:: yaml

   name: geekbench4
   version: 4.2.0
   summary: Cross-Platform Benchmark
   description: |
     Geekbench 4 measures your system's power and tells
     you whether your computer is ready to roar. How
     strong is your mobile device or desktop computer?
     How will it perform when push comes to crunch?
     These are the questions that Geekbench can answer.

   confinement: devmode
   base: core22

   parts:
     geekbench4:
       plugin: dump
       source: https://cdn.geekbench.com/Geekbench-$SNAPCRAFT_PROJECT_VERSION-Linux.tar.gz

   apps:
     geekbench4:
       command: geekbench4

   lint:
     ignore:
       - library

We'll break this file down into its components in the following sections.

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of
human-readable metadata, which is often already available in the project's
own packaging metadata or :file:`README.md` file. This data is used in the
presentation of the application in the Snap Store.

.. code:: yaml

   name: geekbench4
   version: 4.2.0
   summary: Cross-Platform Benchmark
   description: |
     Geekbench 4 measures your system's power and tells
     you whether your computer is ready to roar. How
     strong is your mobile device or desktop computer?
     How will it perform when push comes to crunch?
     These are the questions that Geekbench can answer.

The ``name`` must be unique in the Snap Store. Valid snap names consist of
lower-case alphanumeric characters and hyphens. They cannot be all numbers and
they also cannot start or end with a hyphen.

By value used for the ``version`` is the version number of the released
software.

The ``summary`` cannot exceed 79 characters. You can use a chevron '>' in the
``description`` key to declare a multi-line description.

Base
~~~~

The base keyword declares which :term:`base snap` to use with the project.
A base snap is a special kind of snap that provides a run-time environment
alongside a minimal set of libraries that are common to most applications.

.. code:: yaml

   base: core22

In this example, `core22`_ is used as the base for snap building, and is based
on `Ubuntu 22.04 LTS`_. See :ref:`base-snaps` for more details.

Security model
~~~~~~~~~~~~~~

Snaps are containerised to ensure more predictable application behaviour and
greater security. The general level of access a snap has to the user's system
depends on its level of confinement.

The next section of the :file:`snapcraft.yaml` file describes the level of
:term:`confinement` applied to the running application:

.. code:: yaml

   confinement: devmode

It is best to start creating a snap with a confinement level that provides
warnings for confinement issues instead of strictly applying confinement.
This is done by specifying the ``devmode`` (developer mode) confinement value.
When a snap is in devmode, runtime confinement violations will be allowed but
reported. These can be reviewed by running :command:`journalctl -xe`.

Because devmode is only intended for development, snaps must be set to strict
confinement before they can be published as "stable" in the Snap Store.
Once an application is working well in devmode, you can review confinement
violations, add appropriate interfaces, and switch to strict confinement.

The above example will also work if you change the confinement from ``devmode``
to ``strict``, as you would before a release.

Parts
~~~~~

Parts define what sources are needed to build your application. Parts can be
anything: programs, libraries, or other needed assets, but for this example,
we only need to use one part to handle the tarball containing the ``geekbench``
binary file:

.. code:: yaml

   parts:
     geekbench4:
       plugin: dump
       source: https://cdn.geekbench.com/Geekbench-$SNAPCRAFT_PROJECT_VERSION-Linux.tar.gz

The ``plugin`` keyword is normally used to select a language or
technology-specific plugin that performs the build steps for the project.
However, in this example, the :ref:`dump plugin <the-dump-plugin>` is used to
unpack the file specified by the ``source`` keyword so that its contents can be
included in the snap. The source can be a local or remote zip file, deb file,
or tarball.

In this example we use the value of the ``SNAPCRAFT_PROJECT_VERSION`` environment
variable to refer to the release tarball. This environment variable is derived
from the value of the ``version`` keyword in the `Metadata`_ section, and using
it in this section helps to keep information about the application version in
one place.

Apps
~~~~

Apps are the commands and services that the snap provides to users. Each key
under ``apps`` is the name of a command or service that should be made
available on users' systems.

.. code:: yaml

   apps:
     geekbench4:
       command: geekbench4

The ``command`` specifies the path to the binary to be run. This is resolved
relative to the root of the snap contents.

If the command name matches the name of the snap specified in the top-level
``name`` keyword (see `Metadata`_ above), the binary file will be given the
same name as the snap, as in this example.
If the names differ, the binary file name will be prefixed with the snap name
to avoid naming conflicts between installed snaps. An example of this would be
``geekbench4.some-command``.


Building the snap
-----------------

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/geekbench4

After you have created the :file:`snapcraft.yaml` file (which already exists
in the above repository), you can build the snap by simply executing the
:command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Executed: pull geekbench4
   Executed: build geekbench4
   Executed: stage geekbench4
   Executed: prime geekbench4
   Executed parts lifecycle
   Generated snap metadata
   Created snap package geekbench4_4.2.0_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous``
flag because the snap is not signed by the Snap Store. The ``--devmode`` flag
acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install test-geekbench4_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ test-geekbench4

.. |execname| replace:: test-geekbench4
.. include:: common/removing-cleaning-snap.rst

.. Potentially just refer the reader to another tutorial.
.. include:: common/publishing-snap.rst

Congratulations! You've just built and published your first pre-built binary snap. For a more in-depth overview of the snap building process, see `Creating a snap </t/creating-a-snap/6799>`__.

.. _`Geekbench 4`: https://github.com/snapcraft-docs/geekbench4
