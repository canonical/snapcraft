.. _explanation-interfaces:

Interfaces
==========

A strictly-confined snap is considered untrusted and runs in a restricted environment.
It's only able to access a limited set of resources outside the environment it runs in.
Access to system resources and other snaps is granted on a granular basis using a
mechanism called `interfaces <https://snapcraft.io/docs/interface-management>`_.

For example, a browser without network access doesn't serve its intended purpose.
To that end, snap developers can use the `network interface
<https://snapcraft.io/docs/network-interface>`_ to provide network access to the
browser.

Design
------
.. image:: https://assets.ubuntu.com/v1/59c290a8-snapd-interfaces.png
   :alt: Interface between slots and plugs.

An interface consists of a connection between a slot and a plug. A slot is
the provider of the interface and may be a system resource or access to another snap.
A slot can support multiple plug connections. A plug is the consumer and connects to a
slot through an interface.

Interfaces can be automatically or manually connected. Some interfaces automatically
connect when a snap is installed. Others interfaces don't automatically connect,
especially if they have access to sensitive resources like network control.
See `Supported interfaces <https://snapcraft.io/docs/supported-interfaces>`_ for
details on which interfaces can auto-connect.

Users have the option to manually control interfaces by connecting and disconnecting
them using snapd. See `Interface management
<https://snapcraft.io/docs/interface-management>`_ for details.
