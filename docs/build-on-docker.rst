.. 4158.md

.. _build-on-docker:

Build on Docker
===============

:ref:`Snapcraft <snapcraft-overview>`, the snap-building tool, is designed to use `Multipass <https://community.ubuntu.com/t/installing-multipass-on-linux/8328>`__ and :ref:`bases <base-snaps>` to both simplify the build process and to confine the build environment within a virtual machine. This mostly removes the need to use Docker.

However, Docker can still be used, and is particularly useful when you’re already using Docker within your build and test infrastructure. All you need is a working :file:`snapcraft.yaml` (see :ref:`Creating a snap <creating-a-snap>` for more details).

.. note::
          ℹ Electron apps are built slightly differently, and do not use a :file:`snapcraft.yaml` file. :ref:`Follow this guide <electron-apps>` to build a snap of an Electron app using electron-builder.

To create a snap with Docker, first make sure you have `Docker installed <https://docs.docker.com/install/>`__. You can test that Docker is correctly set up with:

.. code:: bash

   $ docker run hello-world
   [...]
   Status: Downloaded newer image for hello-world:latest

   Hello from Docker!
   [...]

If you don’t see the “Hello from Docker!” message, consult the `Docker documentation <https://docs.docker.com/install/linux/linux-postinstall/>`__ for troubleshooting steps.

Snaps using bases
-----------------

When using a :ref:`base snaps <base-snaps>` other than ``core``, a custom Docker image will need to be built. For details, see :ref:`Creating docker images for snapcraft <creating-docker-images-for-snapcraft>`.

The process for building a ``core18`` compatible image, the default for :ref:`Snapcraft 3.x <release-notes-snapcraft-3-0>`, is outlined below.

Docker images are built from a `Dockerfile <https://docs.docker.com/engine/reference/builder/>`__. This is a script that Docker interprets to assemble whatever is required to generate the image.

The following is an example Dockerfile to build a ``core18`` compatible image:

.. code:: shell

   FROM ubuntu:bionic as builder

   # Grab dependencies
   RUN apt update
   RUN apt dist-upgrade --yes
   RUN apt install --yes \
         curl \
         sudo \
         jq \
         squashfs-tools

   # Grab the core snap from the stable channel and unpack it in the proper place
   # (the 'Snap-CDN: none' header allows building in restricted network
   # environments such as Launchpad builders)
   RUN curl -L -H 'Snap-CDN: none' $(curl -H 'X-Ubuntu-Series: 16' 'https://api.snapcraft.io/api/v1/snaps/details/core' | jq '.download_url' -r) --output core.snap
   RUN mkdir -p /snap/core
   RUN unsquashfs -d /snap/core/current core.snap

   # Grab the core18 snap from the stable channel and unpack it in the proper place
   RUN curl -L -H 'Snap-CDN: none' $(curl -H 'X-Ubuntu-Series: 16' 'https://api.snapcraft.io/api/v1/snaps/details/core18' | jq '.download_url' -r) --output core18.snap
   RUN mkdir -p /snap/core18
   RUN unsquashfs -d /snap/core18/current core18.snap

   # Grab the snapcraft snap from the stable channel and unpack it in the proper place
   RUN curl -L -H 'Snap-CDN: none' $(curl -H 'X-Ubuntu-Series: 16' 'https://api.snapcraft.io/api/v1/snaps/details/snapcraft?channel=stable' | jq '.download_url' -r) --output snapcraft.snap
   RUN mkdir -p /snap/snapcraft
   RUN unsquashfs -d /snap/snapcraft/current snapcraft.snap

   # Create a snapcraft runner
   RUN mkdir -p /snap/bin
   RUN echo "#!/bin/sh" > /snap/bin/snapcraft
   RUN snap_version="$(awk '/^version:/{print $2}' /snap/snapcraft/current/meta/snap.yaml)" && echo "export SNAP_VERSION=\"$snap_version\"" >> /snap/bin/snapcraft
   RUN echo 'exec "$SNAP/usr/bin/python3" "$SNAP/bin/snapcraft" "$@"' >> /snap/bin/snapcraft
   RUN chmod +x /snap/bin/snapcraft

   # Multi-stage build, only need the snaps from the builder. Copy them one at a
   # time so they can be cached.
   FROM ubuntu:bionic
   COPY --from=builder /snap/core /snap/core
   COPY --from=builder /snap/core18 /snap/core18
   COPY --from=builder /snap/snapcraft /snap/snapcraft
   COPY --from=builder /snap/bin/snapcraft /snap/bin/snapcraft

   # Generate locale
   RUN apt update && apt dist-upgrade --yes && apt install --yes sudo snapd locales && locale-gen en_US.UTF-8

   # Set the proper environment
   ENV LANG="en_US.UTF-8"
   ENV LANGUAGE="en_US:en"
   ENV LC_ALL="en_US.UTF-8"
   ENV PATH="/snap/bin:$PATH"
   ENV SNAP="/snap/snapcraft/current"
   ENV SNAP_NAME="snapcraft"
   ENV SNAP_ARCH="amd64"



Dockerfiles for the Snapcraft project, including files that can be built with *snapd* from different channels, can be found on `Snapcraft’s GitHub <https://github.com/snapcore/snapcraft/tree/master/docker>`__ repository.

To build a Docker image, enter the following command from the same location as the saved version of the Dockerfile, which we’ve called ``stable.Dockerfile``:

.. code:: bash

   $ docker build --no-cache -f stable.Dockerfile --label mycustomimage --tag mycustomimage:stable --network host .

When the process has completed, you should be able to see the new image in the output from ``docker images``:

.. code:: bash

   REPOSITORY      TAG       IMAGE ID       CREATED              SIZE
   mycustomimage   stable    76dcf5eafcd2   About a minute ago   882MB

Snaps without bases
-------------------

If your :file:`snapcraft.yaml` file has no ``base`` entry or ``base: core`` defined, you can simply pull down the latest snapcraft image:

.. code:: bash

   $ docker pull snapcore/snapcraft:stable
   [...]
   Status: Downloaded newer image for snapcore/snapcraft:stable

Running a build
---------------

After either building or downloading the snapcraft Docker image, return to the root directory of the project containing your snapcraft.yaml and run snapcraft:

.. code:: bash

   $ docker run -v "$PWD":/build -w /build <IMAGE-NAME> snapcraft

Repleace ``<IMAGE-NAME>`` with either the name of your manually built Docker image, ``mycustomimage:stable`` in our example above, or the downloaded image, such as ``snapcore/snapcraft:stable``.

These options instruct Docker to map the current directory, your project root, to the ``/snapcraft_build`` directory inside the container, and then start the :command:`snapcraft` command (the final command-line argument) from this same location inside the container.

When the snap build completes successfully, you will find a ``.snap`` file in the current directory. You can inspect its contents to ensure it contains all of your application’s assets:

.. code:: bash

   $ unsquashfs -l *.snap

.. note::
          ⚠ Docker may contaminate your project directory with files owned by ``root``, causing *permission denied* errors. Use ``sudo chown --changes --recursive $USER:$USER _project_folder_`` to regain ownership of these files.

Next steps
~~~~~~~~~~

After creating a snap, you should upload it to the `Snap Store <https://snapcraft.io/store>`__. See :ref:`Releasing your app <releasing-your-app>` for further details.
