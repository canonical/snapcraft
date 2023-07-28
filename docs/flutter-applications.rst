.. 18768.md

.. _flutter-applications:

Flutter applications
====================

`Flutter <https://flutter.dev/>`__ is Google’s open source toolkit for
creating applications for mobile, web and desktop platforms.

Snapcraft can be used to package and distribute Flutter applications in a
way that enables convenient installation by users.

The process of creating a snap for a Flutter application builds on standard
Python packaging tools, making it possible to adapt or integrate an
application's existing packaging into the snap building process.

Snaps are defined in a single :file:`snapcraft.yaml` file placed in a
:file:`snap` folder at the root of your project. This YAML file describes
the application, its dependencies and how it should be built.


Getting started
---------------

The following shows the entire :file:`snapcraft.yaml` file for a simple
template project, `super-cool-app`_:

.. code:: yaml

   name: super-cool-app
   version: git
   summary: Super Cool App
   description: Super Cool App that does everything!
   confinement: strict
   base: core18
   grade: stable
   architectures:
     - build-on: [ amd64 ]
     - build-on: [ arm64 ]

   apps:
     super-cool-app:
       command: super_cool_app
       extensions: [flutter-stable]

   parts:
     super-cool-app:
       source: .
       plugin: flutter
       flutter-target: lib/main.dart # The main entry-point file of the application

We'll break this file down into its components in the following sections.

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of
human-readable metadata, which is often already available in the project's
own packaging metadata or :file:`README.md` file. This data is used in the
presentation of the application in the Snap Store.

.. code:: yaml

   name: super-cool-app
   version: '1.0'
   summary: Super Cool App
   description: Super Cool App that does everything!

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

The ``version`` string can be an arbitrary version, such as ``1.0`` in our example, or keywords such as ``git``, to import a version string from a git tag or commit, for instance. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use a chevron '>' in the
``description`` key to declare a multi-line description.

Base
~~~~

The base keyword declares which :term:`base snap` to use with the project.
A base snap is a special kind of snap that provides a run-time environment
alongside a minimal set of libraries that are common to most applications.

.. code:: yaml

   base: core18

In this example, `core18`_ is used as the base for snap building, and is based
on `Ubuntu 18.04 LTS`_. See :ref:`Base snaps <base-snaps>` for more details.

Security model
~~~~~~~~~~~~~~

Snaps are containerised to ensure more predictable application behaviour and
greater security. The general level of access a snap has to the user's system
depends on its level of confinement.

The next section of the :file:`snapcraft.yaml` file describes the level of
:term:`confinement` applied to the running application:

.. code:: yaml

   confinement: strict

It is best to start creating a snap with a confinement level that provides
warnings for confinement issues instead of strictly applying confinement.
This is done by specifying the ``devmode`` (developer mode) confinement value.
When a snap is in devmode, runtime confinement violations will be allowed but
reported. These can be reviewed by running :command:`journalctl -xe`.

Because devmode is only intended for development, snaps must be set to strict
confinement before they can be published as "stable" in the Snap Store.
Once an application is working well in devmode, you can review confinement
violations, add appropriate interfaces, and switch to strict confinement.

In this example, strict confinement is already in use.

Parts
~~~~~

Parts define what sources are needed to build your application. Parts can be
anything: programs, libraries, or other needed assets, but for this example,
we only need to use one part for the *super-cool-app* source code:

.. code:: yaml

   parts:
     super-cool-app:
       plugin: flutter
       source: https://github.com/snapcraft-docs/super-cool-app
       flutter-target: lib/main.dart

The ``plugin`` keyword is used to select a language or technology-specific
plugin that knows how to perform the build steps for the project.
In this example, the :ref:`flutter plugin <the-flutter-plugin>` is used to
build the project.

The ``source`` keyword points to the project source code, which can be a local
directory or remote Git repository. In this case, it refers to the main project
repository.

Apps
~~~~

Apps are the commands and services that the snap provides to users. Each key
under ``apps`` is the name of a command or service that should be made
available on users' systems.

.. code:: yaml

   apps:
     super-cool-app:
       command: super_cool_app
       extensions: [flutter-stable]

The ``command`` specifies the path to the binary to be run. This is resolved
relative to the root of the snap contents.

If your command name matches the snap ``name``, users will be able run the command directly.

If the command name matches the name of the snap specified in the top-level
``name`` keyword (see `Metadata`_ above), the binary file will be given the
same name as the snap, as in this example.
If the names differ, the binary file name will be prefixed with the snap name
to avoid naming conflicts between installed snaps. An example of this would be
``super-cool-app.some-command``.

The ``extensions`` keyword is used to incorporate Flutter's common set of requirements. See :ref:`snapcraft-extensions` for further details.

Building the snap
~~~~~~~~~~~~~~~~~

First, make sure you have installed :ref:`Snapcraft <snapcraft-overview>` and create a new directory for your Flutter project.

Inside that directory, type :command:`snapcraft init`. This creates an additional subdirectory, called :file:`snap` containing a template
:file:`snapcraft.yaml` file.

Edit the created :file:`snapcraft.yaml` to contain the Flutter example shown earlier.

After you’ve created the :file:`snapcraft.yaml`, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   Launched: snapcraft-super-cool-app
   [...]
   Pulling flutter-extension
   [...]
   Building super-cool-app
   [...]
   Staging flutter-extension
   Staging gnome-3-28-extension
   Staging super-cool-app
   Priming flutter-extension
   Priming gnome-3-28-extension
   Priming super-cool-app
   'grade' property not specified: defaulting to 'stable'.
   Snapping |
   Snapped super-cool-app_1.0_amd64.snap

The build process may take some time as both Flutter and the Dart SDK from Flutter are downloaded and installed into the build environment, but they won’t be downloaded again with subsequent builds unless the environment is reset.

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store, or if you’re testing pre-confinement, the ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   sudo snap install super-cool-app_1.0_amd64.snap --dangerous

You can then try it out:

.. code:: bash

   super-cool-app

.. figure:: https://assets.ubuntu.com/v1/f12e5af3-flutter_01.png
   :alt: Running example Flutter application


Removing the snap is simple too:

::

   sudo snap remove super-cool-app

You now have a snap you can deploy and upload to the `Snap Store <https://snapcraft.io/store>`__. See :ref:`Releasing your app <releasing-your-app>` for more details, and to get a deeper insight into the snap building process, start with the :ref:`Snapcraft checklist <snapcraft-checklist>`.

.. _`super-cool-app`: https://github.com/snapcraft-docs/super-cool-app
