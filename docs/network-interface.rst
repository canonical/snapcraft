.. 13124.md

.. _network-interface:

Network Interface
=================

Applications snapped using strict confinement have no network access by default. The ``network`` interface allows all outbound network access from the application as a client. Should the application bind to a port, for example to run a server, then the ``network-bind`` interface should also be used.

Both ``network`` and ``network-bind`` interfaces are automatically connected on installation, and require no additional store review to be used.

::

   apps:
     client:
       command: bin/client-application
       plugs:
         - network
     frontend:
       command: bin/server-application
       plugs:
         - network
         - network-bind

Further network-related interfaces are typically not required, unless the application needs to interrogate or control network interfaces, or manage the local firewall. Most of these are not automatically connected, but can be manually connected by the end user. In addition, the publisher may request auto-connection of these interfaces via a snapcraft forum thread.

See :ref:`the-network-interface`.
