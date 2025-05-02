.. _how-to-debug-a-snap:

Debug a snap
============

The execution and internals of a snap are complex, making debugging as much an art as a
craft. This page contains general advice, methods, and strategies for diagnosing and
remedying a snap that has problems building or running properly.

.. important::

    The content on this page is a work in progress. Some guidance might be out-of-date
    across different cores and versions of Snapcraft.


Where to look for errors
------------------------

Here are the common sources of errors when crafting a snap.


Build environment
~~~~~~~~~~~~~~~~~

Snaps are built to run on top of a :ref:`base snap <explanation-bases>` runtime. This
base is provided by an automatically-installed snap. Currently, the most widely-used
core images are based on Ubuntu 22.04 LTS and Ubuntu 24.04 LTS.


Missing libraries
^^^^^^^^^^^^^^^^^

Most apps need additional libraries added to the snap in order to function correctly. As
the developer of the app, you're best placed to know which libraries the snap needs
staged during build.

Sometimes when a snap is initially built, libraries are missing because they weren't
explicitly specified by the developer. There's a couple of ways to bundle required
libraries in a snap, both of which are covered below, but more details can be found in
:ref:`how-to-manage-dependencies`.


Staged packages
^^^^^^^^^^^^^^^

It's common to bundle required libraries in snaps with the ``stage-packages`` key in the
project file. These are standard package names from the Ubuntu repository used by the
base snap. To find packages in the Ubuntu archive, search for them in the `Ubuntu
Archive <https://packages.ubuntu.com>`_. Make sure to choose the Ubuntu version used by
your base snap when searching.

Don't include glibc or libc6 in your list of staged packages. Doing so is unnecessary as
the base snap contains those libraries already, and bundling them into your snap can
cause unexpected behaviour.


Individual libraries
^^^^^^^^^^^^^^^^^^^^

Some app developers already have vendored libraries that they've tested well with their
app. To bundle these libraries into a snap, add them to its ``/lib`` directory. The
directory is included in ``LD_LIBRARY_PATH`` and should be found successfully by your
app when the snap is installed on a device.


Interfaces
~~~~~~~~~~

The `list of interfaces <https://snapcraft.io/docs/supported-interfaces>`_ details the
capabilities that each brings. When an interface is omitted, this may result in the app
misbehaving. Consult the list to identify the necessary interfaces required by their
app.

The `snappy-debug <https://snapcraft.io/snappy-debug>`_ tool was created for debugging
missing interface connections. It helps identify missing interfaces by reporting app
security failures, and will make suggestions on how to improve the snap.

To debug with the tool:

1. In a terminal, run ``snappy-debug``.
2. Launch the snapped app in another terminal instance.
3. Run the snapped app until a failure occurs.
4. Examine the output from snappy-debug.

Typically the output will contain messages about failed attempts to access system
resources, and suggest additional interfaces which should be specified. If so, add any interfaces listed and rebuild the snap.


Inspect the snap's contents
---------------------------

To help speed up testing, it's possible to open a shell within the build environment to
check the state of a build, view logs, probe the value of environment variables, locate
missing binaries and install missing dependencies.

The following commands enable you to step into this encapsulated environment:

.. list-table::
    :header-rows: 1

    * - Debug command
      - What it does
    * - ``--shell``
      - Builds your snap to the lifecycle step prior to the one specified, and opens a
        shell into the environment.

        FFor example, running ``snapcraft prime --shell``
        will run up to the stage step, then open a shell.
    * - ``--shell-after``
      - Builds your snap to the lifecycle step specified, and opens a shell into the
        environment.

        For example, running ``snapcraft prime --shell-after`` will run up to the prime
        step, then open a shell.
    * - ``--debug``
      - Opens a shell inside the environment after an error occurs. If Snapcraft
        encounters an error it will open a shell within the virtualized build
        environment.

        This enables you to view logs within the environment, check the value of
        environment variables, locate missing binaries, and install missing
        dependencies. You can even edit the project file outside of the shell, and then
        run Snapcraft within, to continue the build.

For example, with a snap named snapcraft-test, to open a shell just before the prime
step, run:

.. code-block:: bash

    snapcraft prime --shell

The last line of the output contains a bang (#), indicating you're in a shell:

.. terminal::

    Using 'snap/snapcraft.yaml': Project assets will be searched for from
    the 'snap' directory.
    Launching a VM.
    Launched: snapcraft-test
    [...]
    Pulling part-test
    Building part-test
    Staging part-test
    snapcraft-test #

If a build has already progressed past the stage specified, first clean the build or the
part, then rebuild:

.. code-block:: bash

    snapcraft clean
    snapcraft build --shell


.. _iterate-on-the-build-lifecycle:

Iterate on the build lifecycle
------------------------------

Build issues are linked to the stage of the :ref:`lifecycle
<explanation-parts-lifecycle>` that Snapcraft is working through when it generates an
error. The most common problems associated with each step are outlined below.


Before the build
~~~~~~~~~~~~~~~~

Errors in the project file typically occur early, before any processing, and they're
usually easy to resolve.

For example, the following error is related to a missing key:

.. terminal::

    Issues while validating snapcraft.yaml: 'adopt-info' is a required property or
    'version' is a required property:

This issue is caused by a mandatory key, ``version``, not being defined.

However, ``version`` isn't actually mandatory when paired with ``adopt-info``, because
``adopt-info`` pushes version details into Snapcraft from its specified part. This isn't
processed until later in the build, which means any error in adopt-info isn't generated
until the prime step:

.. terminal::

    Failed to generate snap metadata: 'adopt-info' refers to part 'mypart', but that
    part is lacking the 'parse-info' property.

To resolve this particular missing key, make sure your part includes ``parse-info`` or
runs a command to define the version details, such as ``craftctl set-version``.

For more information on the keys affected by this error, see :ref:`Configure package
information <how-to-configure-package-information-from-appstream>`.


Build step
~~~~~~~~~~

Errors in this step are only generated by projects building their own binaries from
source code.

Any issues that occur are likely to be similar to those associated with compiling the
project outside of Snapcraft, and it can help to first build manually, or be familiar
with the part that's failing to build, before updating your snap.

As an example:

.. terminal::

    Package ncursesw was not found in the pkg-config search path.
    Perhaps you should add the directory containing 'ncursesw.pc'
    to the PKG_CONFIG_PATH environment variable
    No package 'ncursesw' found
    pkg-config: exit status 1

This example is easily rectified by adding libncursesw5-dev to the
``build-packages`` in the part that's failing to build, which includes it as a
dependency.

.. code-block:: yaml

    build-packages:
      - libncursesw5-dev

In this way the solution is just like resolving dependencies with other build tools.

For more details on package names and build dependencies, see
:ref:`how-to-manage-dependencies`.


Stage step
~~~~~~~~~~

Errors in this step are synonymous with missing dependencies in any runtime environment.

Missing elements are typically libraries, and sometimes binaries, that an app needs to
run correctly. Errors manifest at runtime, or when a library should be accessed, and
they are thrown by the app rather than the snap.

A Git client, for example, might not invoke the ``git`` command until it needs to.
Only then will its absence become apparent, and only if Git isn't installed on the host
system.

For example:

.. terminal::

    Unable to successfully call git binary. If git is not in $PATH then please set the
    config variable git-binary-file-path

The solution is to add the packages for these missing dependencies to the
``stage-packages`` key in the affected part:

.. code-block:: yaml

    stage-packages:
      - git


Debug with snapd
----------------

The ``snap`` command itself has many diagnostic features that can help with debugging
runtime and configuration errors. `Debugging snaps
<https://snapcraft.io/docs/debug-snaps>`_ in the snapd documentation covers how and when
to use them.
