.. 6847.md

.. _choosing-a-security-model:

Choosing a security model
=========================

Snaps are *containerised* to ensure predictable application behaviour and to provide greater security.

Containerisation is enforced at the system level, using Discretionary Access Controls (DAC), Mandatory Access Control (MAC) via AppArmor, Seccomp kernel system call filtering (which limits the system calls a process may use), and *cgroups* device access controls for hardware assignment.

When :ref:`creating a snap <creating-a-snap>`, the extent of a snap’s containment is defined by the ``confinement`` keyword within its :file:`snapcraft.yaml` file:

.. code:: yaml

   confinement: strict

Confinement can be one of the following:

- ``devmode``: enables system access and logging only while creating a snap
- ``strict``: denies all system access other than through a snap’s :ref:`interfaces <interface-management>`
- ``classic``: permits full system-level access, analogous to a *deb* or *rpm* installation.

The vast majority of snaps should be able to attain *strict* confinement, but there are three specific requirements that can typically only be resolved with *classic* confinement:

#. access to arbitrary filesystem paths outside of ``$HOME`` and /media
#. run other programs that cannot be determined at build time
#. perform privileged tasks not yet covered by interfaces (see below)

As a a result of these requirements, publishing a classic snap requires a :ref:`review and approval process <process-for-reviewing-classic-confinement-snaps>`, alongside the manual addition of ``--classic`` to the *snap* command when the snap is installed.

   ℹ See :ref:`Snap confinement <snap-confinement>` for general details on confinement levels and the implications for the user.

From *devmode* to *strict*
--------------------------

During the :ref:`build and testing phase <creating-snapcraft-yaml>` of snap development, it helps to have confinement set to ``devmode``, as shown in the example above. This is also the default when using the \`snapcraft init’ command to initialise a new project directory.

A snap built with *devmode* can be :ref:`published and released <releasing-your-app>`, but not to its stable :term:`channel`, and it will not appear in search results. To release to the stable channel, its confinement needs to be either ``strict`` (in the majority of cases), or ``classic`` when a manual exception to its confinement needs to be made

To confine your application, return to your snapcraft.yaml file and change the confinement value from ``devmode`` to ``strict``:

.. code:: yaml

   confinement: strict

.. note::
          ℹ If you are working with an :ref:`Electron app <electron-apps>`, you will not have a snapcraft.yaml file. Set the ``confinement`` `property <https://www.electron.build/configuration/snap>`__, under the top-level ``snap`` key in your package.json file. This defaults to ``strict``.

You will also likely need to specify some interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port. You can find a list of interfaces and their intended purpose in the :ref:`reference documentation <supported-interfaces>`.

The interfaces are specified as a list of “``plugs``” in your snapcraft.yaml file, alongside your ``command`` definition. For example, if your application needs access to the Internet and to the user’s home directory:

.. code:: yaml

   apps:
     offlineimap:
       command: bin/offlineimap
       plugs: [home, network]

If you have multiple ``command`` definitions, you will need to provide separate ``plugs`` definitions for each.

Now that your snapcraft.yaml is updated for confinement, rebuild your snap. This is a quick process when only the confinement method has changed.

:ref:`Install and test <iterating-over-a-build>` your rebuilt snap. If your app continues to work as expected, you’re ready for publishing in the Snap Store.

See :ref:`releasing your app <releasing-your-app>` for details on how to publish your snap.

Debugging
~~~~~~~~~

If your app has failed to start or behaves incorrectly, you may be missing some interfaces. Check ``journalctl -xe`` for a possible explanation, then refer to the interfaces in the :ref:`reference documentation <supported-interfaces>`. Add any missing interfaces to your ``plugs`` definition, rebuild your snap, and test again.

If no explanation can be found, ask for assistance on the `Snapcraft Forum <https://forum.snapcraft.io/c/snap>`__. Be sure to include any relevant details, such as the contents of log files and any error messages printed on the terminal.
