.. _explanation-interfaces:

About interfaces
================

A strictly-confined snap is considered untrusted and runs in a restricted environment.
It's only able to access a limited set of resources outside the environment it runs in.
Access to system resources and other snaps is granted on a granular basis using a
mechanism called :external+snap:ref:`interfaces
<explanation-interfaces-all-about-interfaces>`.

For example, a browser without network access doesn't serve its intended purpose. To
that end, snap developers can use the :external+snap:ref:`interfaces-network-interface`
to provide network access to the browser.


Design
------

.. image:: https://assets.ubuntu.com/v1/59c290a8-snapd-interfaces.png
   :alt: Interface between slots and plugs.

A slot can be connected to one or more plugs through an interface. The slot provides
this interface and access to either a system resource or a resource from another snap.
The plug connects to the provided interface and consumes the resource.

Some interfaces connect automatically when a snap is installed, while others, such as
those that have access to sensitive resources, need to be connected manually. See
:external+snap:ref:`ref-index_interfaces` for details on
which interfaces connect automatically.

Users can control interfaces manually by connecting and disconnecting them with snapd.
See :external+snap:ref:`explanation-interfaces-all-about-interfaces` for details.
