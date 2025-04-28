.. _reference-build-environment-options:

Build environment options
=========================

Snapcraft can optionally use the following arguments to modify the build environment.


.. list-table::
   :header-rows: 1
   :widths: 2 1 1

   * - Snapcraft Argument
     - Description
     - Notes

   * - ``--destructive-mode``
     - **Destructive mode**. Designed to be used in scenarios where additional
       provisioning of the build environment is required. This is not recommended
       because the build *could* contaminate the host build environment.
     - See :ref:`destructive mode <reference-build-environment-options-destructive>`.

   * - ``--use-lxd``
     - Builds the snap using `LXD <https://linuxcontainers.org/lxd/introduction/>`_
       rather than Multipass. This can potentially reduce resource usage, especially
       from a VM.
     - Requires LXD. For more information, see :ref:`how-to-select-a-build-provider`.

   * - ``--http-proxy <http-proxy>``
     - Configures HTTP proxy. Snapcraft will honor the ``http_proxy`` environment
       variable as well.
     - None.

   * - ``--https-proxy <https-proxy>``
     - Configures HTTPS proxy. Snapcraft will honor the ``https_proxy`` environment
       variable as well.
     - None.

   * - ``--bind-ssh``
     - Bind the :file:`~/.ssh` directory to the local build instance.
     - Requires LXD or Multipass. Only available when building core22 snaps.

   * - ``--ua-token <token>``
     - Configure the build environment with ESM using specified UA token.
     - Requires LXD or Multipass. Only works for snaps built on core22.

   * - ``--enable-manifest``
     - Add the build manifest to the snap package in :file:`snap/manifest.yaml`. This
       contains the specific sources and packages used to build the snap and allows the
       Snap Store to `automatically check your snap for security issues
       <https://snapcraft.io/blog/introducing-developer-notifications-for-snap-security-updates>`_.
       This option can also be used by setting the environment variable
       ``SNAPCRAFT_BUILD_INFO=1``.
     - Snaps built on Launchpad will have this set
       automatically. For snaps newer than core22, the command-line flag is deprecated
       and removed. The environment variable will still work.


.. _reference-build-environment-options-destructive:

Destructive mode
~~~~~~~~~~~~~~~~

Destructive mode builds the snap directly on the machine where Snapcraft is run without
launching a build container. It is not recommended to use this option because Snapcraft
cannot control the build environment. It is especially not recommended to use a personal
machine or in a shared environment, where operations like adding package repositories
may be disruptive.

The build environment should match the snap base. For example, a core24 snap should be
built inside of an Ubuntu 24.04 environment.

For core22 and newer snaps, the user must have root-level permissions so that they can
do operations such as installing snaps and apt packages and adding package repositories
without needing to use sudo.

While destructive mode can be used in CI to save time, using an OCI image or a LXD
container is recommended. See `snapcraft rocks
<https://github.com/canonical/snapcraft-rocks>`_ for OCI images that can be used with
Docker to build snaps. For projects in GitHub, the `action-build
<https://github.com/canonical/action-build>`_ workflow is a good option.
