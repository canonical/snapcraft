.. 7983.md

.. _environment-variables:

Environment variables
=====================

Environment variables are widely used across Linux to provide convenient access to system and application properties.

Both :ref:`Snapcraft <snapcraft-overview>` and snapd consume, set, and pass-through specific environment variables to support building and running snaps.

See below for the various environment variables available to snap applications. For environment variables connected to Snapcraft, see :ref:`Parts environment variables <parts-environment-variables>`.

Snap specific environment variables
-----------------------------------

List environment variables
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each snap runs in a custom environment specifically made for it. To get an overview of the variables in it, you can open a shell as the snap and run the ``env`` command.

.. code:: bash

   $ snap run --shell <snap>.<command>
   $ env
   XDG_VTNR=1
   SSH_AGENT_PID=5543
   XDG_SESSION_ID=2
   SNAP_USER_COMMON=/home/<user>/snap/<snap>/common
   SNAP_LIBRARY_PATH=/var/lib/snapd/lib/gl:
   SNAP_COMMON=/var/snap/<snap>/common
   [...]

Alongside the many system-specific variables, this environment will include the following:

.. list-table::
   :header-rows: 0

   * - `SNAP <environment-variables-snap_>`__
     - `SNAP_ARCH <environment-variables-snap-arch_>`__
   * - `SNAP_COMMON <environment-variables-snap-common_>`__
     - `SNAP_DATA <environment-variables-snap-data_>`__
   * - `SNAP_EUID <environment-variables-snap-euid_>`__
     - `SNAP_INSTANCE_NAME <environment-variables-snap-instance-name_>`__
   * - `SNAP_INSTANCE_KEY <environment-variables-snap-instance-key_>`__
     - `SNAP_LIBRARY_PATH <environment-variables-snap-library-path_>`__
   * - `SNAP_NAME <environment-variables-snap-name_>`__
     - `SNAP_REAL_HOME <environment-variables-snap-real-home_>`__
   * - `SNAP_REVISION <environment-variables-snap-revision_>`__
     - `SNAP_SAVE_DATA <environment-variables-snap-save-data_>`__
   * - `SNAP_UID <environment-variables-snap-uid_>`__
     - `SNAP_USER_COMMON <environment-variables-snap-user-common_>`__
   * - `SNAP_USER_DATA <environment-variables-snap-user-data_>`__
     - `SNAP_VERSION <environment-variables-snap-version_>`__
   * - `HOME <environment-variables-home_>`__
     - `PATH <environment-variables-path_>`__


.. _environment-variables-snap:

SNAP
~~~~

Directory where the snap is mounted. This is where all the files in your snap are visible in the filesystem. All of the data in the snap is read-only and cannot be changed.

Typical value: ``/snap/hello-world/27``


.. _environment-variables-snap-arch:

SNAP_ARCH
~~~~~~~~~

CPU architecture of the running system.

Typical value ``amd64``

Other values are: ``i386``, ``armhf``, ``arm64``.


.. _environment-variables-snap-common:

SNAP_COMMON
~~~~~~~~~~~

Directory for system data that is common across revisions of a snap.

This directory is owned and writable by ``root`` and is meant to be used by background applications (daemons, services). Unlike ``SNAP_DATA`` this directory **is not** backed up and restored across snap refresh and revert operations.

Typical value: ``/var/snap/hello-world/common``


.. _environment-variables-snap-data:

SNAP_DATA
~~~~~~~~~

Directory for system data of a snap.

This directory is owned and writable by ``root`` and is meant to be used by background applications (daemons, services). Unlike ``SNAP_COMMON`` this directory is backed up and restored across ``snap refresh`` and ``snap revert`` operations.

Typical value ``/var/snap/hello-world/27``


.. _environment-variables-snap-euid:

SNAP_EUID
~~~~~~~~~

This variable contains the *effective* user ID (euid) of the user running the snap instance. See also `SNAP_UID <environment-variables-snap-uid_>`__.

For this variable to be exposed by a snap, the snap developer will need to include the following :ref:`assumes <snapcraft-top-level-metadata-assumes>` value:

.. code:: yaml

   assumes: [snap-uid-envvars]

Requires *snapd* 2.59+.


.. _environment-variables-snap-instance-name:

SNAP_INSTANCE_NAME
~~~~~~~~~~~~~~~~~~

The name of snap instance, including instance key if one is set (snapd 2.36+).

For example snap ``hello-world`` with instance key ``foo`` has instance name equal to ``hello-world_foo``.

Typical value: ``hello-world``


.. _environment-variables-snap-instance-key:

SNAP_INSTANCE_KEY
~~~~~~~~~~~~~~~~~

Instance key if one was set during installation or empty (snapd 2.36+).

For example instance ``hello-world_foo`` has an instance key ``foo``.

Typical value: none


.. _environment-variables-snap-library-path:

SNAP_LIBRARY_PATH
~~~~~~~~~~~~~~~~~

Directory with additional system libraries. This variable is used internally by snapcraft.

The value is always ``/var/lib/snapd/lib/gl:`` Please note the colon at the end of that value, the variable is a colon-separated list.

The referenced directory is typically empty unless Nvidia proprietary drivers are in use.


.. _environment-variables-snap-name:

SNAP_NAME
~~~~~~~~~

The name of the snap as specified in the :file:`snapcraft.yaml` file.

Typical value: ``hello-world``


.. _environment-variables-snap-real-home:

SNAP_REAL_HOME
~~~~~~~~~~~~~~

The vanilla ``HOME`` environment variable before snapd-induced remapping, refer
to `this forum thread <acquire original HOME variable_>`_ for more information.

Available `since snapd 2.46 <https://github.com/snapcore/snapd/pull/9189/commits/37d0a229>`__.


.. _environment-variables-snap-revision:

SNAP_REVISION
~~~~~~~~~~~~~

Revision of the snap, as allocated by the Snap Store on upload or as allocated by snapd for locally installed snaps.

The Snap Store assigns monotonic revisions to each upload of a given snap. Snapd uses Snap Store revisions if accompanying assertions are available or uses a locally generated number. Locally generated numbers are prefixed with ``x`` to distinguish them from Snap Store uploads.

Typical value: ``27`` or ``x1``


.. _environment-variables-snap-save-data:

SNAP_SAVE_DATA
~~~~~~~~~~~~~~

This variable is only exposed on :term:`Ubuntu Core` systems, and was introduced with snapd 2.57.

It points to a snap-specific location on the ubuntu-save partition where the snap is allowed to store persistent files (like certificates or configuration files) that will survive a `factory reset`_ of the Ubuntu Core device.

See `ubuntu-save`_ in the Ubuntu Core documentation for more details on storage layout with this specific partition.


.. _environment-variables-snap-uid:

SNAP_UID
~~~~~~~~

This variable contains the user ID (uid) of the user running this snap instance. See also `SNAP_EUID <environment-variables-snap-euid_>`__.

For this variable to be exposed by a snap, the snap developer will need to include the following :ref:`assumes <snapcraft-top-level-metadata-assumes>` value:

.. code:: yaml

   assumes: [snap-uid-envvars]

Requires *snapd* 2.59+.


.. _environment-variables-snap-user-common:

SNAP_USER_COMMON
~~~~~~~~~~~~~~~~

Directory for user data that is common across revisions of a snap.

Unlike ``SNAP_DATA``, data present in this directory is not backed up or restored across ``snap refresh`` and ``snap revert`` operations. The directory is suitable for large data that the application can access even if it was made or modified by a future version of a snap.

Typical value ``/home/zyga/snap/hello-world/common``


.. _environment-variables-snap-user-data:

SNAP_USER_DATA
~~~~~~~~~~~~~~

Directory for user data.

This directory is backed up and restored across ``snap refresh`` and ``snap revert`` operations.

Typical value: ``/home/zyga/snap/hello-world/27``

The final number there is ``$SNAP_REVISION``.


.. _environment-variables-snap-version:

SNAP_VERSION
~~~~~~~~~~~~

The version string as specified in the :file:`snapcraft.yaml` file.

Typical value ``6.3``

Generic variables
-----------------


.. _environment-variables-home:

HOME
~~~~

For non-classic snaps, this environment variable is re-written to ``SNAP_USER_DATA`` by snapd so that each snap appears to have a dedicated home directory that is a subdirectory of the real home directory.

For classic confinement snaps, the value remains unchanged.

Typical value: ``/home/_user_name_/snap/_snap_name_/_snap_revision_`` (e.g. ``/home/zyga/snap/hello-world/27``)


.. _environment-variables-path:

PATH
~~~~

This environment variable is re-written by snapd so that it is consistent with the view of the filesystem presented to snap applications.

The value is always:

-  For non-classic confinement snaps:

   ::

      $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games

-  For classic confinement snaps: ``/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games``

.. _`acquire original HOME variable`: https://forum.snapcraft.io/t/19475
.. _`factory reset`: https://ubuntu.com/core/docs/recovery-modes#environment-variables-heading--factory
.. _`ubuntu-save`: https://ubuntu.com/core/docs/storage-layout#environment-variables-heading--save
