.. 9357.md

.. _the-personal-files-interface:

The personal-files interface
============================

The ``personal-files`` interface provides access to the specified files in the user’s home. This interface gives privileged access to the user’s data.

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

**Transitional**: no

**Attributes**:

* ``read`` (plug): list of files and/or directories for read-only access (eg, ‘``read: [ $HOME/.file-read, $HOME/.dir-read ]``’
* ``write`` (plug): list of files and/or directories for read/write access (eg, ‘``write: [ $HOME/.file-write, $HOME/.dir-write ]``’

Specifying a directory in ``read`` and ``write`` allows access to the directory and all files under it.

Requires snapd version *2.37+*.

Snaps by default have access to everything under ``$SNAP_USER_DATA`` (eg, ``~/snap/<snap name>/<revision>``) and ``$HOME`` is set to ``$SNAP_USER_DATA`` for non-daemon commands. As a result, snaps may freely read and write to dot files in their ``$HOME`` (ie, ``~/snap/<snap name>/<revision>/...``).

The ``personal-files`` interface is typically used to provide read-only access to top-level hidden data directories within a user’s real home directory in order to support importing data from existing applications where the snap is the clear owner of the target directory.

For distribution via the `Snap store <https://snapcraft.io/store>`__, consumers of this interface require an approved `snap declaration <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. For acceptance, you will need to make a descriptive interface reference, as used by ``snap connections|interfaces|connect|disconnect`` commands.

For example, if a *foo* application is being packaged as a snap and its publisher wants the snap to import an existing configuration from ``~/.config/foo`` into ``$SNAP_USER_DATA/.config/foo`` (ie, ``$HOME/.config/foo`` within the snap’s runtime environment or ``~/snap/foo/<revision>/.config/foo``) on the host), the :file:`snapcraft.yaml` file could include the following:

.. code:: yaml

   name: foo
   ...
   plugs:
     dot-config-foo:
       interface: personal-files
       read:
       - $HOME/.config/foo

   apps:
     foo:
       plugs:
       - dot-config-foo
       ...

Note, when declaring an instance of the ``personal-files`` plug as above, it should be named with a descriptive name that indicates to a user what access it grants. In this case, the name ``dot-config-foo`` is used to reflect the access to ``~/.config/foo``.

With the above built snap, you would then be able to use the following to enable access to personal files:

.. code:: bash

   $ snap connect foo:dot-config-foo

..

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
