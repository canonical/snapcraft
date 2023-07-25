.. 33017.md

.. _how-snapcraft-builds-snaps:

How Snapcraft builds snaps
==========================

The Snapcraft tool can perform a variety of operations:

-  Snap creation operations like build, pack, prime, stage, etc.
-  Store management operations like login, register, sign-build, etc.
-  Extensions-specific operations like expand-extensions and list-extensions.
-  Other operations.

These operations translate into practical tasks like:

-  It can build snaps locally or send remote tasks to Launchpad.
-  It allows the developer to register and login into the Snap Store.
-  It can upload snaps into the Snap Store.
-  It can promote snaps to different channels.

The list of available global options and commands can be checked with:

.. code:: bash

   snapcraft help


.. _how-snapcraft-builds-snaps-snapcraft:

snapcraft.yaml
--------------

Snaps are created using a build recipe defined in a file called :file:`snapcraft.yaml` file.

When the snapcraft tool is executed on the command line, it will look for the file in the current project work directory, either in the top-level folder or the *f* sub-folder. If the file is found, snapcraft will then parse its contents and progress the build toward completion.

:file:`snapcraft.yaml` is a configuration file written in the YAML language, with stanzas defining the application structure and behavior. When snapcraft runs and parses this file, it will use the declared information to build the snap. For developers more familiar with the traditional Linux build systems, the process is somewhat similar to the use of a Makefile or an RPM spec file.

You can create the snapcraft.yaml file manually in your project directory, or you can run the snapcraft init command, which will create a template file you can then populate with the required information.

.. code:: bash

   snapcraft init

A minimal valid snapcraft.yaml file that can be built into a working snap requires three stanzas:

-  Metadata.
-  Confinement level.
-  Build definition.


.. _how-snapcraft-builds-snaps-definitions:

Main definitions inside snapcraft.yaml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is no one way for how a snap ought to be assembled. However, most snapcraft.yaml files have the same common elements, including a number of mandatory declarations. Below is a short list of these keys, which will be further explained in the Examples sections later in the tutorial.

-  **Metadata** - describes the snap functionality and provides identifiers by which the snap can be cataloged and searched in the Snap Store.
-  **Security confinement** - describes the level of security of the snap.
-  **Base** - describes which set of libraries the snap will use for its functionality. The base also defines the operating system version for the snap build instance in the virtual machine or container launched by Snapcraft. For instance, base: core18 means that Snapcraft will launch an Ubuntu 18.04 virtual machine or container, the set of tools and libraries used inside the snap will originate from the Ubuntu 18.04 repository archives, and the snap applications will “think” they are running on top of an Ubuntu 18.04 system, regardless of what the actual underlying Linux distribution is.
-  **Parts** - describes the components that will be used to assemble the snap.
-  **Apps** - describes the applications and their commands that can be run in an installed snap.

It is important to note several additional details:

-  A snap may contain one or more parts.
-  A snap may contain one or more applications
-  Parts can be pre-assembled binaries or they may be compiled as part of the build process.
-  The parts section of the snapcraft.yaml file uses Snapcraft build system or language-specific plugins to simplify the build process.
-  The parts section may also include a list of :ref:`build packages <build-and-staging-dependencies>` (build-packages) that will be used to create the snap applications but will not be included in the final snap. For instance, gcc or make.

The parts section may also include a list of :ref:`stage packages <build-and-staging-dependencies>` (stage-packages) that will be used by the snap’s applications at runtime, e.g.: python-bcrypt. These will be obtained from the repository archives in the build instance.


.. _how-snapcraft-builds-snaps-build:

Snapcraft build lifecycle
-------------------------

Snaps are :ref:`built <parts-lifecycle>` in several steps, collectively known as the “lifecycle”:

-  **Pull** - At this step of the snap build process, Snapcraft downloads or retrieves the components needed to build the relevant part. For instance, if source points to a Git repository, the pull step will clone that repository.
-  **Build** - Snapcraft constructs the part from the previously pulled components. Since the snap ecosystem supports multiple types of applications (C, Java, Go, Rust, Python, etc.), the build definition also needs to include a specification on how to construct the part. This is done by declaring a :ref:`Snapcraft plugin <snapcraft-plugins>`. Parts are processed linearly, unless there is a dependency order declared.
-  **Stage** - Snapcraft copies the built parts into the staging area. Parts are not ordered at this point, and there might be an additional level of processing to ensure the snap contains the required files, and that there are no conflicts between parts. This is an advanced topic beyond the scope of this tutorial.
-  **Prime** - Snapcraft copies the staged components into the priming area, where the files will be placed in their final locations (folder and files path hierarchy) for the resulting snap. The prime step is similar to the stage step, but it may exclude certain components from the stage step.
-  **Pack** - Snapcraft packs the assembled components in the prime directory into a single archive.


.. _how-snapcraft-builds-snaps-output:

Snapcraft build output
----------------------

The artifact of a successful Snapcraft build run is a snap file, which is itself a compressed Squashfs archive distinguished by the .snap suffix.

A snap may contain one or more files that allow the applications to run without reliance on the underlying host system’s libraries. A snap will contain one or more applications, daemons, configuration files, assets like icons, and other objects.

Typically, the content of a snap will resemble a Linux filesystem layout:

.. code:: text

   drwxr-xr-x 10 igor igor  4096 Jun 10  2020 ./
   drwxrwxrwx 14 igor igor 16384 Oct 17 16:40 ../
   drwxr-xr-x  2 igor igor  4096 Jun 10  2020 bin/
   drwxr-xr-x 10 igor igor  4096 Jun 10  2020 etc/
   -rw-r--r--  1 igor igor 14 Jun 10  2020 flavor-select
   drwxr-xr-x  3 igor igor  4096 Jun 10  2020 lib/
   drwxr-xr-x  2 igor igor  4096 Jun 10  2020 lib64/
   drwxr-xr-x  3 igor igor  4096 Jun 10  2020 meta/
   drwxr-xr-x  3 igor igor  4096 Jun 10  2020 snap/
   drwxr-xr-x  7 igor igor  4096 Jun 10  2020 usr/
   drwxr-xr-x  3 igor igor  4096 Feb 26  2018 var/

The end user can examine the contents of a snap by extracting the snap archive:

.. code:: bash

   unsquashfs <file>.snap
