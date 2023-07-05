.. 33076.md

.. _intermediate-snapcraft-yaml-example:

Intermediate snapcraft.yaml example
===================================

We will examine a complete snapcraft.yaml file for an application called *wethr*, a command line weather tool.

Below, we will look at blocks of code that were not previously covered in the :ref:`Basic snapcraft.yaml example <basic-snapcraft-yaml-example>`, or those that differ significantly from what we already discussed.

The full snapcraft.yaml file is available on the `project GitHub page <https://github.com/snapcrafters/wethr/blob/master/snap/snapcraft.yaml>`__. The contents may change and slightly differ from the example shown below.

.. code:: yaml

   name: wethr
   summary: Command line weather tool.
   description: |
     Get current weather

   adopt-info: wethr
   base: core20
   grade: stable
   confinement: strict

   architectures:
     - build-on: amd64
     - build-on: armhf
     - build-on: arm64

   apps:
     wethr:
       command: bin/wethr
       plugs:
         - network

   parts:
     wethr:
       plugin: npm
       npm-node-version: "10.14.1"
       source: https://github.com/twobucks/wethr.git
       source-type: git
       override-pull: |
         snapcraftctl pull
         last_committed_tag="$(git describe --tags --abbrev=0)"
         last_committed_tag_ver="$(echo ${last_committed_tag} | sed 's/v//')"
         last_released_tag="$(snap info wethr | awk '$1 == "latest/beta:" { print $2 }')"
         # If the latest tag from the upstream project has not been released to
         # beta, build that tag instead of master.
         if [ "${last_committed_tag_ver}" != "${last_released_tag}" ]; then
           git fetch
           git checkout "${last_committed_tag}"
         fi
         snapcraftctl set-version "$(git describe --tags | sed 's/v//')"
       build-packages:
         - git
         - sed

The metadata, base, and confinement declarations are rather similar to our basic example, but with some notable differences:

-  The wethr snap does not explicitly declare the snap version. This is something that will be explained below.
-  The wethr snap uses core20 as its base.
-  The wethr snap uses strict confinement instead of devmode.


.. _intermediate-snapcraft-yaml-example-adopt:

adopt-info
----------

This keyword instructs Snapcraft to “adopt” metadata information using :ref:`external metadata <using-external-metadata>` sources. Such use can be useful for CI systems, where the declarations in the snapcraft.yaml file can be obtained from scripts rather than manually.

-  There are multiple ways that information can be obtained.
-  Multiple metadata fields can be populated using this keyword.

In this example, the wethr snap application version is obtained from the Git repository release tag. This is done two stages:

-  adopt-info keyword instructs Snapcraft to populate fields not explicitly specified.
-  In the parts section at the end of the snapcraft.yaml file (which will be discussed below):

   -  A portion of the default snap lifecycle definition is manually :ref:`overridden <override-build-steps>` (override-pull).
   -  A set of commands in the BASH shell syntax is used to derive the version string.
   -  The version string is set using the snapcraftctl scriptlet.

.. code:: yaml

   adopt-info: wethr

Alternatively, in this particular example, the version field could also be manually defined, e.g.: version: ‘1.5’.


.. _intermediate-snapcraft-yaml-example-grade:

grade
-----

The optional grade keyword defines the quality level of the snap. Two levels are available: devel and stable. Snaps with the devel grade level cannot be uploaded to the stable channel in the Snap Store.

.. code:: yaml

   grade: stable


.. _intermediate-snapcraft-yaml-example-architectures:

Architectures
-------------

This section defines the target :ref:`architectures <architectures>` for which the snap should be built. It requires the build system that is running the Snapcraft tool to be able to compile and build the snap for the listed platforms.

.. code:: yaml

   architectures:
   - build-on: amd64
   - build-on: armhf
   - build-on: arm64


.. _intermediate-snapcraft-yaml-example-build:

Build definition
----------------

While largely similar to the yt-dlp example, the wethr application does introduce some notable differences in the build definition section. We will discuss the parts section first.


.. _intermediate-snapcraft-yaml-example-parts:

The parts definition
~~~~~~~~~~~~~~~~~~~~

The parts definition consists of the following lines of code:

.. code:: yaml

   parts:
     wethr:
       plugin: npm
       npm-node-version: "10.14.1"
       source: https://github.com/twobucks/wethr.git
       source-type: git
       override-pull: |
         snapcraftctl pull
         last_committed_tag="$(git describe --tags --abbrev=0)"
         last_committed_tag_ver="$(echo ${last_committed_tag} | sed 's/v//')"
         last_released_tag="$(snap info wethr | awk '$1 == "latest/beta:" { print $2 }')"
         # If the latest tag from the upstream project has not been released to
         # beta, build that tag instead of master.
         if [ "${last_committed_tag_ver}" != "${last_released_tag}" ]; then
           git fetch
           git checkout "${last_committed_tag}"
         fi
         snapcraftctl set-version "$(git describe --tags | sed 's/v//')"
       build-packages:
         - git
         - sed

The wethr snap also only has one part. However, here, it is built using the npm plugin, which is a Snapcraft plugin designed to simplify the building of Node.js and JavaScript-based applications.

-  **plugin**: This block defines the use of the Snapcraft :ref:`npm plugin <the-npm-plugin>` that will perform various language-specific commands in the background. The npm plugin creates parts that use Node.js and/or the JavaScript package manager npm. The plugin declaration has several sub-sections:

   -  **npm-node-version:** defines the specific version of Node to be used.
   -  **source**: defines the URL or a path of the application code that needs to be downloaded for the build. It can be a local or remote path, and can refer to a directory tree, a compressed archive or a revision control repository.
   -  **source-type**: defines the type of the online source. This allows the plugin to perform relevant source-specific actions to successfully complete the download of the necessary data for the part.

-  **override-pull**: opens a multi-line block inside which BASH-syntax commands are used to perform operations that cannot be satisfied by the default Snapcraft lifecycle pull operation. In the wethr example, the listed commands are used to derive the right version of the application, and set it using the snapcraftctl scriptlet.
-  **build-packages**: defines the list of tools and libraries that are required to successfully build or compile the part. The build packages are obtained from the repository archives that match the snap base, and need to be written in the syntax that can be correctly interpreted by the apt package manager. For instance, a foo build package from the Ubuntu 20.04 archive would be installed (apt install foo) in the snap build environment during the build lifecycle. In this case, the snap needs the git tool to retrieve the sources from the Git repository (GitHub) and the sed tool to perform the string search and replace action on the commit tag.


.. _intermediate-snapcraft-yaml-example-apps:

The apps definition
~~~~~~~~~~~~~~~~~~~

The apps build definition consists of the following lines of code:

.. code:: yaml

   apps:
     wethr:
       command: bin/wethr
       plugs:
         - network

The wethr example has a single application - wethr. Other snaps may have multiple sub-applications or executables.

-  **command**: defines the path to the executable (relative to the snap) and arguments to use when this application runs.
-  **plugs**: defines the list of interfaces to which the app will have access to. This enables the intended application functionality. In this specific case, the wethr snap will be allowed access to the network interface, which is not available by default under strict confinement, and thus be able to retrieve the weather information from online sources.
