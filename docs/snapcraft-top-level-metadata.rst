.. 8334.md

.. _snapcraft-top-level-metadata:

Snapcraft top-level metadata
============================

The top-level keys and values in :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` provide the snap build process, and the store, with the overarching details of a snap.

   See :ref:`Snapcraft app metadata <snapcraft-app-and-service-metadata>` and :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>` for details on how apps and parts are configured within :file:`snapcraft.yaml`.

Top-level details include a snap’s name, version and description, alongside operational values such as its confinement level and supported architecture.

adopt-info
----------

Type: ``string`` (*optional*)

Incorporate external metadata via the referenced part.

See :ref:`Using external metadata <using-external-metadata>` for more details.

architectures
-------------

Type: ``list[object]`` (*optional*)

List of build and run architectures.

For more details, see :ref:`Architectures <architectures>`.


.. _snapcraft-top-level-metadata-assumes:

assumes
~~~~~~~

Type: ``list[string]`` (*optional*)

A list of features that must be supported by the core in order for this snap to install. For example to make the snap only installable on certain recent version of snapd(like 2.38) you can specify:

.. code:: yaml

   assumes:
   - snapd2.38

Other potential values for *assumes* include:

- ``common-data-dir``: support for common data directory across revisions of a snap
- ``snap-env``: support for the “Environment:” feature in snap.yaml
- ``command-chain``: support for the “command-chain” feature for apps and hooks in snap.yaml
- ``kernel-assets``: support for kernel assets in :ref:`gadget.yaml <gadget-snaps-specification>`, such as to include volume content in the style ``$kernel:ref``


.. _snapcraft-top-level-metadata-base:

base
~~~~

Type: ``string`` (*optional*)

A snap of type ``base`` to be used as the execution environment for this snap. See :ref:`Base snaps <base-snaps>` for further details.

.. list-table:: Values
   :header-rows: 1

   * - **Name**
     - **Description**
   * - ``bare``
     - Empty base snap, useful for fully statically linked snaps and testing
   * - ``core``
     - Ubuntu Core 16
   * - ``core18``
     - Ubuntu Core 18
   * - ``core20``
     - Ubuntu Core 20
   * - ``core22``
     - Ubuntu Core 22


.. _snapcraft-top-level-metadata-compression:

compression
~~~~~~~~~~~

Type: ``string`` (*optional*)

Sets the compression type for the snap. Can be ``xz`` or ``lzo``. Defaults to ``xz`` when not specified.

Snaps are compressed using *xz* data compression by default. This offers the optimal performance to compression ratio for the majority of snaps.

However, there are certain types of snap, such as large desktop applications, that can benefit from using LZO compression. Snaps compressed with *lzo* are slightly larger but can decompress quicker, reducing the time it takes for freshly installed or refreshed snaps to launch.

To specify *lzo* compression, set ``compression: lzo`` in your snap’s :file:`snapcraft.yaml` file and rebuild your snap, as shown in the following example:

.. code:: yaml

   name: test-snapcraft-lzo
   base: core18
   version: "0.1"
   summary: Test LZO snap
   description: Test LZO snap
   grade: stable
   confinement: strict

   # this line enables LZO compression for the snap
   compression: lzo

   parts:
     my-part:
       plugin: nil

   apps:
     lzo-things:
       command: bin/something

confinement
-----------

Type: ``enum`` (*optional*)

Determines if the snap should be restricted in access or not.

Possible values are ``strict`` (for no access outside of declared ``interfaces`` through ``plugs``), ``devmode`` (for unrestricted access) or ``classic``. For more information, refer to :ref:`Confinement <snap-confinement>`.

Examples: ``strict``, or ``devmode``

contact
-------

Type: ``list[string] | string`` (Introduced: Snapcraft 5.0 *optional*)

A contact for the snap in the form of a URL or email address.

description
-----------

Type: ``string`` (*mandatory*)

Multi-line description of the snap.

A more in-depth look at what your snap does and who may find it most useful.

donation
--------

Type: ``list[string] | string`` (Introduced: Snapcraft 5.0 *optional*)

A link or list of links to receive donations for the snap.


.. _snapcraft-top-level-metadata-epoch:

epoch
~~~~~

type: ``integer`` (*optional*)

Controls when users receive a configuration-breaking application release.

Applications and their data formats are constantly evolving, and this requires applications to periodically break data compatibility with older versions. When this happens, applications and users often need to carefully manage data migration from one version to another, and this is where epochs can help. By default, snaps have an epoch of ‘0’. When a new version breaks data compatibility with this old version, incrementing the epoch in the new release stops those old users automatically refreshing to the new version.

See :ref:`Snap epochs <snap-epochs>` for further details.


.. _snapcraft-top-level-metadata-grade:

grade
~~~~~

Type: ``enum`` (*optional*)

Defines the quality ``grade`` of the snap.

Can be either ``devel`` (i.e. a development version of the snap, so not to be published to the ``stable`` or ``candidate`` channels) or ``stable`` (i.e. a stable release or release candidate, which can be released to all channels).

A snap of ``type`` ``app`` (default) cannot be set to ``stable`` if the ``base`` is not on a stable channel.

Example: ``[stable`` or ``devel``]


.. _snapcraft-top-level-metadata-hooks:

hooks
~~~~~

Type: ``list[dict]`` (*optional*)

Hooks permit executable files to run within a snap’s confined environment when a certain action occurs.

By default, hooks run with no plugs. If a hook needs more privileges, you can use this top-level ``hooks`` attribute:

.. code:: yaml

   hooks: # Top-level YAML attribute, parallel to `apps`
     configure: # Hook name, corresponds to executable name
       plugs: [network] # Or any other plugs required by this hook

See :ref:`Snapcraft hook support <snapcraft-hook-support>` for more details.

issues
------

Type: ``list[string] | string`` (Introduced: Snapcraft 5.0 *optional*)

A link or list of links to report issues for the snap.


.. _snapcraft-top-level-metadata-icon:

icon
~~~~

Type: ``string`` (*optional*)

Path to icon image that represents the snap in the snapcraft.io store pages and other graphical store fronts.

*Note that the* `desktop menu <https://en.wikipedia.org/wiki/Start_menu>`__ *does not use this icon. It uses the icon in the* ``.desktop`` *file of the application.*

It is a relative path to a ``.png``/``.svg`` file from the source tree root. The `recommended <https://snapcraft.io/docs/restrictions-on-screenshots-and-videos-in-snap-listings24>`__ size is 256x256 pixels. Aspect ratio needs to be 1:1. Image size can vary from 40x40 to 512x512 px and the file size should not be larger than 256 KB.

Examples: ``_package_name_.svg``, or ``snap/gui/logo.png``


.. _snapcraft-top-level-metadata-layout:

layout
~~~~~~

Type: ``list[dict]`` (*optional*)

Layouts modify the execution environment of a :ref:`strictly-confined <snap-confinement>` snap.

With layouts, you can make elements in ``$SNAP`` , ``$SNAP_DATA`` , ``$SNAP_COMMON`` accessible from locations such as ``/usr`` , ``/var`` and ``/etc`` . This helps when using pre-compiled binaries and libraries that expect to find files and directories outside of locations referenced by ``$SNAP`` or ``$SNAP_DATA`` .

See :ref:`Snap layouts <snap-layouts>` for more details.

Example:

.. code:: yaml

   layout:
     /var/lib/foo:
       bind: $SNAP_DATA/var/lib/foo
     /usr/share/foo:
       bind: $SNAP/usr/share/foo
     /etc/foo.conf:
       bind-file: $SNAP_DATA/etc/foo.conf


.. _snapcraft-top-level-metadata-license:

license
~~~~~~~

Type: ``string`` (*optional*)

A license for the snap in the form of an `SPDX expression <https://spdx.org/licenses/>`__ for the license.

In the legacy Snapcraft syntax (not using the ``base`` key), this key is only available :ref:`through the passthrough key <using-in-development-features-in-snapcraft-yaml>`.

Currently, only `SPDX 2.1 expressions <https://spdx.org/spdx-specification-21-web-version>`__ are supported. A list of supported values are also available at `snapd/licenses.go at master · snapcore/snapd <https://github.com/snapcore/snapd/blob/master/spdx/licenses.go>`__.

For “or later” and “with exception” license styles refer to `the Appendix IV of the SPDX Specification 2.1 <https://spdx.org/spdx-specification-21-web-version#h.jxpfx0ykyb60>`__.

Examples: ``GPL-3.0+``, ``MIT``, ``Proprietary``

name
----

Type: ``string`` (*mandatory*)

The identifying name of the snap.

It must start with an ASCII character and can only contain letters in lower case, numbers, and hyphens, and it can’t start or end with a hyphen. The name must be unique if you want to :ref:`publish to the Snap Store <releasing-your-app>`.

For help on choosing a name and registering it on the Snap Store, see :ref:`Registering your app name <registering-your-app-name>`.

Example: ``my-awesome-app``

package-repositories
--------------------

Type: ``list[dict]`` (*optional*)

Adds package repositories as sources for build-packages and stage-packages, including those hosted on a PPA, the Personal Package Archive, which serves personally hosted non-standard packages.

See :ref:`Snapcraft package repositories <snapcraft-package-repositories>` for more details.

Example:

.. code:: yaml

   package-repositories:
     - type: apt
       components: [main]
       suites: [xenial]
       key-id: 78E1918602959B9C59103100F1831DDAFC42E99D
       url: http://ppa.launchpad.net/snappy-dev/snapcraft-daily/ubuntu

passthrough
-----------

Type: ``type[object]`` (*optional*)

Attributes to passthrough to ``snap.yaml`` without validation from snapcraft.

See :ref:`Using development features in snapcraft <using-in-development-features-in-snapcraft-yaml>` for more details.

source-code
-----------

Type: ``string`` (Introduced: Snapcraft 5.0 *optional*)

A link to the source of the snap (i.e.; the repository containing ``snapcraft.yaml``).

summary
-------

Type: ``string`` (*mandatory*)

Sentence summarising the snap.

Max len. 78 characters, describing the snap in short and simple terms.

Example: ``The super cat generator``

system-usernames
----------------

Type: ``dict`` (*optional*)

Common example is ``snap_daemon: shared`` to use a daemon user, see :ref:`system-usernames` for more details.

title
-----

Type: ``string`` (*optional*)

The canonical title of the application, displayed in the software centre graphical frontends.

Max length 40 characters.

In the legacy Snapcraft syntax (not using the ``base`` key), this key is only available :ref:`through the passthrough key <using-in-development-features-in-snapcraft-yaml>`.

Example: ``My Awesome Application``

type
----

Type: ``enum`` (*optional*)

The type of snap, implicitly set to ``app`` if not set.

For more details, see: :ref:`gadget <gadget-snaps>`, :ref:`kernel <the-kernel-snap>`, :ref:`base <base-snaps>`.

Example: ``[app|core|gadget|kernel|base]``

version
-------

Type: ``string`` (*mandatory*, unless using ``adopt-info``)

A user facing version to display.

This field is mandatory unless version information is provided by ``adopt-info`` . See :ref:`Using external metadata <using-external-metadata>` for details.

Max len. 32 chars. Needs to be wrapped with single-quotes when the value will be interpreted by the YAML parser as non-string.

Examples: ``'1'``, ``'1.2'``, ``'1.2.3'``, ``git`` (will be replaced by a ``git describe`` based version string)


.. _snapcraft-top-level-metadata-plugs-and-slots-for-an-entire-snap:

Plugs and slots for an entire snap
----------------------------------

Plugs and slots for an :ref:`interface <supported-interfaces>` are usually configured per-app or per-daemon within :file:`snapcraft.yaml`. See :ref:`Snapcraft app metadata <snapcraft-app-and-service-metadata>` for more details. However, ``snapcraft.yaml`` also enables global *plugs* and *slots* configuration for an entire snap:

.. _snapcraft-top-level-metadata-plugs:

plugs
-----

Type: ``dict`` *(optional)*

These plugs apply to all ``apps`` and differs from **apps.<app-name>.plugs** in that the type is in a ``dict`` rather than a ``list`` format, ``:``\ (colon) must be postfixed to the interface name and shouldn’t start with ``-``\ (dash-space).

plugs.<plug-name>
-----------------

Type: ``dict`` *(optional)*

A set of attributes for a plug.

Example: ``read`` attribute for the ``home`` interface.

plugs.<plug-name>.<attribute-name>
----------------------------------

Type: ``string`` *(optional)*

Value of the attribute. Example: ``all`` for ``read`` attribute of the ``home`` interface.

.. _snapcraft-top-level-metadata-slots:

slots
-----

Type: ``dict`` *(optional)*

A set of slots that the snap provides, applied to all the ``apps``.

slots.<slot-name>
-----------------

Type: ``dict`` (*optional*)

A set of attributes of the slot.

slots.<slot-name>.<attribute-name>
----------------------------------

Type: ``dict`` (*optional*)

Value of the attribute.

website
-------

Type: ``string`` (Introduced: Snapcraft 5.0 *optional*)

A link to a product website from the publisher of the snap.
