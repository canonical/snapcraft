.. 22938.md

.. _debug-snaps-with-snap-try:

Debug snaps with snap try
=========================

The ``snap try`` command installs a snap from its unpackaged components within a directory. After installation, most changes to the components in that directory will immediately by visible in the installed snap.

Ordinarily, snaps cannot be modified. They are distributed as, and executed from, read-only SquashFS files whose integrity is guaranteed via `assertions <https://snapcraft.io/docs/assertions>`__ and the `store <https://snapcraft.io/docs/using-the-snap-store>`__.

But it’s sometimes useful to be able to experiment with a snap locally, to help debug an issue, or to make changes to a snap when you don’t have upstream access to the packaging process. This is when ``snap try`` is useful.

-  If you only have access to the packaged snap itself, you first need to `unsquash the snap <debug-snaps-with-snap-try-unsquash_>`__ and then `use snap try to test it <debug-snaps-with-snap-try-snaptry_>`__.
-  If, instead, you are a developer with access to the source code of the snap, you first need to `use snapcraft try to build an unpacked snap <debug-snaps-with-snap-try-snapcrafttry_>`__, and then `use snap try to test it <debug-snaps-with-snap-try-snaptry_>`__.


.. _debug-snaps-with-snap-try-unsquash:

Unsquashing a snap
------------------

In order to use ``snap try``, you first need to get the unpacked contents of the snap. The first step is to get the snap itself. The most practical source of snaps is the snap store, where a snap can be downloaded with the ``snap download`` command:

.. code:: bash

   $ snap download hello-world
   Fetching snap "hello-world"
   Fetching assertions for "hello-world"
   Install the snap with:
   snap ack hello-world_29.assert
   snap install hello-world_29.snap

The download includes the snap itself and a signed set of `assertions <https://snapcraft.io/docs/assertions>`__ from the store to validate the snap’s default state.

Alternatively, *.snap* files for any installed snaps can be found at ``/var/lib/snapd/snaps/``, from where they can be copied across to your current working directory.

To uncompress the SquashFS *.snap* file, use ``unsquashfs <snap filename>``:

.. code:: bash

   $ unsquashfs hello-world_29.snap
   Parallel unsquashfs: Using 8 processors
   6 inodes (6 blocks) to write
   [===========================|] 6/6 100%
   created 6 files
   created 4 directories
   created 0 symlinks
   created 0 device

The files associated with the snap can now be found in the \`squashfs-root’ directory. You can use `snap try <debug-snaps-with-snap-try-snaptry_>`__ to install this unpacked snap.


.. _debug-snaps-with-snap-try-snaptry:

Using snap try
--------------

Running ``snap try <directory>`` installs an unpacked snap using a bind mount.

.. code:: bash

   $ snap try squashfs-root
   hello-world 6.4 mounted from /home/user/squashfs-root
   $ which hello-world
   /snap/bin/hello-world

Most changes now made to files in the ``squashfs-root`` folder will be immediately reflected in the installed snap. This can be helpful when debugging an application within a snap, or the snap itself.

   ⓘ Certain changes, such as adjusting a snap’s interfaces or confinement will not be reflected in the installed snap until after a reinstall. These changes include:

   -  changes to snap interfaces, such as adding or removing a plug
   -  changes to layouts
   -  changes to a snap’s confinement

Using the above hello-world snap, for example, we could edit the ``bin/echo`` script to change its output without rebuilding or remounting the snap:

.. code:: bash

   $ hello-world
   Hello world!
   $ sed -i 's/World/Everyone/g' /home/user/squashfs-root/bin/echo
   $ hello-world
   Hello Everyone!


.. _debug-snaps-with-snap-try-snapcrafttry:

Using snapcraft try
-------------------

When developing a snap with :ref:`snapcraft <snapcraft-overview>`, the ``snapcraft try`` command can be used in combination with ``snap try`` to quickly test a snap and fix issues.

The ``snapcraft try`` command runs through the build process to the completion of the *prime* stage (see :ref:`Parts lifecycle <parts-lifecycle>` for further details). It then exposes the resultant *prime* directory to the snapcraft directory, even from within a virtual machine or container.

This *prime* directory includes all the staged components of a snap, which can then be installed and tested with the ``snap try <prime directory>`` command.

The following, example, will build a *hello-world* snap within :ref:`LXD <build-options>` and offer its *prime* directory locally:

.. code:: bash

   $ snapcraft try --use-lxd
   Pulling hello-world
   + snapcraftctl pull
   Building hello-world
   + snapcraftctl build
   + cp --archive --link --no-dereference . /root/parts/hello-world/install
   Staging hello-world
   + snapcraftctl stage
   Priming hello-world
   + snapcraftctl prime
   You can now run `snap try /home/user/hello-world/prime`.

The above snap can then be installed and tested with `snap try <debug-snaps-with-snap-try-snaptry_>`__ and the *prime* directory as its target:

.. code:: bash

   $ snap try /home/user/hello-world/prime
   hello-world 0.1 mounted from /home/user/hello-world/prime

For further help on testing and debugging a snap, see :ref:`Iterating over a build <iterating-over-a-build>` and :ref:`Debugging snaps <debugging-snaps>`.
