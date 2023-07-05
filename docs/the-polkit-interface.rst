.. 28408.md

.. _the-polkit-interface:

The polkit interface
====================

The ``polkit`` interface provides daemons with the permission to use the `polkit authorisation manager <https://www.freedesktop.org/software/polkit/docs/latest/polkit.8.html>`__ (*polkitd*) to make access control decisions for requests from unprivileged clients.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-polkit-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

**Attributes**:

-  **action-prefix** (plug, mandatory): indicate that all actions published by the snap are equal to the action prefix or match ``${action-prefix}.*`` .

To perform polkit authorisation checks, a daemon needs to do two things:

1. Install a .policy file to ``$SNAP/meta/polkit/${plug_name}.*/policy`` describing the actions it will use (codifying the type of administrative access a user might be granted). Snapd will install the policy file when the plug is connected.
2. Before performing administrative work on behalf of a client app, make a ``CheckAuthorization`` D-Bus call to polkitd to ask if they have access. The D-Bus call passes a string action ID describing the access, and a “subject” struct describing the client application.

There are two primary ways a daemon can describe the subject of the check:

1. For D-Bus daemons they can use a ``system-bus-name`` subject, sending the unique bus name of the client app.
2. For non-D-Bus daemons, they can use a ``unix-process`` subject, sending the process ID (as retrieved through ``SO_PEERCRED`` or ``SCM_CREDENTIALS``).

See https://snapcraft.io/docs/proposal-add-polkit-and-polkit-agent-interfaces-to-snapd for the original interface proposal and reasoning.

Code examples
-------------

.. code:: yaml

   plugs:
    polkit:
     action-prefix: org.example.foo

   apps:
    app:
     command: foo
     plugs: [polkit]

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/polkit.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/polkit.go
