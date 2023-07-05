.. 1074.md

.. _the-content-interface:

The content interface
=====================

The ``content`` interface allows sharing code and data from a slot on a *producer* snap to one or more plugs on *consumer* snaps. It’s often used to share common themes and icons from a producer snap, for example, to many snapped applications.

Sharing happens at the filesystem level, which means anything that can be expressed as a file can be shared. This includes executables, libraries and data files, but also sockets.


.. _the-content-interface-example:

Example
-------

The `Yaru MATE Icons <https://github.com/ubuntu-mate/icon-theme-yaru-mate-snap>`__ snap is a good producer snap example, letting other applications access the wonderful MATE icon theme. But there are many other producer snaps too, including several for `GTK Common Themes <https://snapcraft.io/gtk-common-themes>`__ and `KDE Frameworks <https://snapcraft.io/kde-frameworks-5-core18>`__ for better application integration with the desktop.

.. note::


          See :ref:`interface-management` and :ref:`supported-interfaces` for further details on how interfaces are used.

--------------


.. _the-content-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no, unless connecting to snaps from the same publisher.

**Attributes**:

* **source (slot)**: allows multiple directories to be exposed separately rather than grouped together
* **read (slot)**: *read-only* paths to be exposed to a consuming snap
* **write (slot)**: *read and write* paths to be exposed to a consuming snap
* **target (plug)**: path in consuming snap to find producer snap’s files
* **default-provider (plug)**: name of preferred producer snap (``<SNAP>``)
* **content (slot and plug)**: an arbitrary identifier for content type. Defaults to either local slot name or local plug name for slot/plug definitions respectively.
* **interface (slot and plug)**: snapd interface name (must be ``interface: content``)


.. _the-content-interface-sharing-content:

Sharing content
---------------

By default, when multiple directories are shared from a producer snap, or when multiple slots are connected to a single plug, the shared content is merged under the ``target`` path of the consuming path’s plug definition. This behaviour can be modified with the ``source`` attribute.

Read, write and target should start with either ``$SNAP``, ``$SNAP_DATA`` or ``$SNAP_COMMON`` to refer to the designated directory. See :ref:`environment-variables` for details on where these point to on the filesystem.

The *content identifier* specified by the consuming snap (plug) must match the *content* attribute of the producer snap (slot).

At a very basic level, the content interface enables one directory, file or socket to appear in a place where another snap can access it.

Each example below involve two snaps: the first provides some content (using a content *slot*) while the second consumes that content (using a content *plug*).

In all of the cases we see a small set of attributes defined on the particular interface:

- the producer declares which path can be read, using either the ``read`` attribute for read-only, or the ``write`` attribute for both read and write permissions
- the consumer uses the ``target`` attribute to define where the content should become available at runtime. - both the producer and consumer use an arbitrary ``content`` attribute to describe the content. This attribute must match on both sides for the connection to happen.


.. _the-content-interface-using-source:

Using source
~~~~~~~~~~~~

The ``source`` attribute presents one or more sub-directories, shared from a slot to a plug, beneath the plug’s ``target`` path. Adding the ``source`` attribute ensures that sub-directories, shared from one or more producer snaps, are presented separately to the consumer snap beneath its ``target`` path.

When multiple slots are connected to the same plug *and* they share directories with the same name, those directories are given unique names with the following pattern: ``<directory>``, ``<directory>-2``, ``<directory>-3``, ``<directory>-x``. The names of shared directories with unique names are retained, as defined by the slot.

With the following example, directories from the producer snap are shared in corresponding directories beneath the *consumer* snap’s ``target`` path:

**producer/snapcraft.yaml**:

.. code:: yaml

   slots:
     _slot_name_:
       interface: content
       content: executables
       source:
         read:
           - $SNAP/bin

**consumer/snapcraft.yaml**:

.. code:: yaml

   plugs:
     _plug_name_:
       interface: content
       content: executables
       target: $SNAP/shared-bin

With the above configuration, the consumer snap could implement a part to run an executable from the following path:

.. code:: bash

   $SNAP/shared-bin/bin/<executable-name>

When more than one slot is connected to the same plug, the ``bin`` directory for the new connection will be incremented:

.. code:: bash

   $SNAP/shared-bin/bin-2/<executable-name>

Directory names are preserved after a reboot.


.. _the-content-interface-read-only:

Read-only content sharing
-------------------------

Read-only content sharing is ideal for executables and files related to global graphical themes and images.

Sharing an executable
---------------------

When the following two interfaces are connected, the *consumer* snap can invoke executables from ``$SNAP/extra-bin``:

**producer/snapcraft.yaml**:

.. code:: yaml

   slots:
     _slot_name_:
       interface: content
       content: executables
       read:
         - $SNAP/bin

**consumer/snapcraft.yaml**:

.. code:: yaml

   plugs:
     _plug_name_:
       interface: content
       content: executables
       target: $SNAP/extra-bin

The directory can be added to ``PATH`` in the wrapper script, if desired, and the directory can also be inspected by any applications that wish to check if the extra executables are available (they can then fail gracefully).

Sharing a C-level library
-------------------------

A consumer snap can link to libraries shared by a producer snap:

**producer/snapcraft.yaml**:

.. code:: yaml

   slots:
     lib0-1604:
       interface: content
       content: lib0-1604
       read:
         - $SNAP/lib

**consumer/snapcraft.yaml**:

.. code:: yaml

   plugs:
     lib0-1604:
       interface: content
       content: lib0-1604
       target: $SNAP/extra-libs

After :ref:`connecting the interface <interface-management>`, the *consumer* snap can link to libraries from ``$SNAP/extra-libs``. The directory can be added to ``LD_LIBRARY_PATH`` in the wrapper script if desired.

The value of the ``content`` attribute can be anything, but it is good practice to follow the form ``nameAPI-BUILDENV`` to remind slot consumers of the API level and build tools used. This naming convention is also *required* when sharing content between snap publishers.

In the above example:

- ``0`` indicates API level 0
- ``1604`` denotes Ubuntu 16.04 LTS toolchain and libraries were used within the build environment

API and BUILDENV can be anything that is meaningful to the provider and consumers. For example, the GNOME content snap uses ``gnome-3-26-1604`` to denote the full GNOME 3.26 platform libraries and supporting files built on Ubuntu 16.04 LTS.


.. _the-content-interface-identifier:

Content identifier obligations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The content identifier attribute identifies a mostly-immutable compatibility contract (API/ABI or similar) between the snap providing the corresponding content and the snaps consuming it.

The providing snap must preserve backward compatibility for the content provided under a given identifier.

Equally, updates to consuming snaps cannot strongly depend on changes of the identified content from updates to the providing snaps.

There is no support in *snapd* to synchronise updates between consuming and providing snaps. Compatibility breaking changes need to happen under a different content identifier.


.. _the-content-interface-default:

Default provider
----------------

The optional ``default-provider`` attribute can be used to set to the name of a snap offering a corresponding content slot:

**consumer/snapcraft.yaml**

.. code:: yaml

   plugs:
     lib0-1604:
       interface: content
       content: lib0-1604
       target: $SNAP/extra-libs
       default-provider: lib01604

If the system does not contain a snap providing a matching slot, installing a consumer snap with a default-provider will trigger the automatic installation of the named provider snap (from *snapd 2.32*). If the named snap is already installed, the absence of a matching slot will instead trigger an update of the named provider snap (from *snapd 2.53*). The plug and slot will be auto-connected assuming the :ref:`auto-connection mechanism <the-interface-auto-connection-mechanism>` is configured properly for this.

For example, a snap consuming the GNOME content snap for GNOME 3.26 can set ``default-provider`` to ``gnome-3-26-1604``.


.. _the-content-interface-writable:

Sharing writable data
---------------------

Sharing writable data can be used to share data files, and *UNIX sockets*, between a group of snaps. This allows for the creation of a simple form of IPC between them.

Sharing writable files (from *snapd 2.19.1*):

**producer/snapcraft.yaml**:

.. code:: yaml

   slots:
     _slot_name_:
       interface: content
       content: writable-data
       write:
         - $SNAP_DATA

**consumer/snapcraft.yaml:**

.. code:: yaml

   plugs:
     _plug_name_:
       interface: content
       content: writable-data
       target: $SNAP_DATA

Sharing UNIX sockets (from *snapd 2.19.1*):

**producer/snapcraft.yaml**:

.. code:: yaml

   slots:
     _slot_name_:
       interface: content
       content: socket-directory
       write:
         - $SNAP_DATA

**consumer/snapcraft.yaml**:

.. code:: yaml

   plugs:
     _plug_name_:
       interface: content
       content: socket-directory
       target: $SNAP_DATA

When the two interfaces are connected the *consumer* snap can see the socket in ``$SNAP_DATA``.


.. _the-content-interface-details:

Technical details
-----------------

The content interface is implemented via an interplay between two systems: `AppArmor <https://wiki.ubuntu.com/AppArmor>`__ and bind mounts.

By default, the AppArmor sandbox allows *writes* to ``$SNAP_DATA`` and *reads* from ``$SNAP`` (see :ref:`environment-variables` for details).

The content interface takes advantage of this feature to map data from other locations to either ``$SNAP`` or ``$SNAP_DATA``.

A bind mount is then created to link ``$SNAP`` in one snap (e.g. from ``/snap/my-snap/1234/content``) to an empty directory in the other snap (e.g., to ``/snap/my-other-snap/4321/incoming-content``).

The same can be done for particular files, if desired, but it requires a pair of interfaces for each file and is more cumbersome.


.. _the-content-interface-code:

Code examples
~~~~~~~~~~~~~

The previously mentioned `Yaro MATE Icons <https://snapcraft.io/icon-theme-yaru-mate>`__ snap is a good example of how this interface can be used to share media with other snaps. Its snapcraft.yaml can be found here: https://github.com/ubuntu-mate/icon-theme-yaru-mate-snap/blob/main/snap/snapcraft.yaml

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/content.go
