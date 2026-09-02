.. _reference-crystal-plugin:

Crystal plugin
==============

The Crystal plugin builds parts whose sources are written in the Crystal programming
language.


Keys
----

This plugin provides the following unique keys for core20 snaps.


crystal-channel
~~~~~~~~~~~~~~~

**Type**: string

**Default**: ``latest/stable``

The Snap Store channel to install the Crystal snap from.


crystal-build-options
~~~~~~~~~~~~~~~~~~~~~

**Type**: list of strings

The options to pass to ``shards build``.


Dependencies
------------

For core20 snaps, this plugin installs the Crystal snap and the following system packages:

* git
* make
* gcc
* pkg-config
* libssl-dev
* libxml2-dev
* libyaml-dev
* libgmp-dev
* libpcre3-dev
* libevent-dev
* libz-dev


How it works
------------

#. Build the Crystal project with ``shards build``, taking into account any build options
   declared with ``crystal-build-options``.
#. Stage the Crystal project's binary and dependencies in ``/.bin``.


Example
-------

The test suite in Snapcraft has a `crystal-hello
<https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/v2/snaps/crystal-hello>`_
snap built with the Crystal plugin.
