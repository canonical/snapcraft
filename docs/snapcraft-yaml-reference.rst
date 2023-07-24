.. 4276.md

.. _snapcraft-yaml-reference:

Snapcraft.yaml reference
========================

This page arranges the same material found in :ref:`The snapcraft.yaml schema <the-snapcraft-yaml-schema>` as a single page.

.. note::

   **NOTE TO EDITORS**

   This is work in progress.

   This document is reference material, where possible, the attribute should include the version it was introduced or deprecated in. Links to more in-depth explanations for these resources are welcome as well as links to reasons for the deprecation.

adopt-info
----------
*optional*

Incorporate external metadata via the referenced part.

**Type:** ``string``

See :ref:`Using external metadata <using-external-metadata>` for more details.

..
.. Note: aliases was deprecated in deprecation-notice-5
..

apps
----
A map of app-names representing entry points to run for the snap.

**Type:** ``dict``

apps.<app-name>
---------------
The name exposed to run a program inside the snap.

**Type:** ``dict``

If ``<app-name>`` is the same as ``name``, the program will be invoked as ``app-name``. However, if they differ, the program will be exposed as ``<snap-name>.<app-name>``.

.. _snapcraft-yaml-activates-on:

apps.<app-name>.activates-on
----------------------------
*optional*

A list of names that the snap exposes as slots that can be used to activate it
via D-Bus. Each name is automatically added to the slots for the snap.

**Type:** ``list[string]``

This keyword is useful when creating services that are activated by other
applications or services.

See also :ref:`bus-name <snapcraft-yaml-bus-name>`, and :ref:`services-and-daemons` for more information.

apps.<app-name>.adapter
-----------------------
Controls the creation of an env variable wrapper. **Type** ``enum`` Can be one of the following:

- ``none``
- ``full`` *(default)*

Snapcraft normally creates a wrapper holding common environment variables. Disabling this could be useful for minimal base snaps without a shell, and for statically linked binaries with no use for an environment.

.. _snapcraft-yaml-after:

apps.<app-name>.after
---------------------
Lists the applications a daemon is to be started after.

**Type:** ``list[string]``

Requires *daemon* to be set in app metadata. See also ``before`` (below) and :ref:`services-and-daemons` for more details.

apps.<app-name>.autostart
-------------------------
The name of the autostart ``.desktop`` file.

**Type:** ``string``

The desktop file is placed in ``SNAP_USER_DATA/.config/autostart``, and the application is started using the app’s command wrapper. See :ref:`snapcraft-parts-metadata` for further details.

apps.<app-name>.before
----------------------
Lists the applications a daemon is to be started before.

**Type:** ``list[string]``

Requires ``daemon`` to be set in app metadata. See also :ref:`snapcraft-yaml-after` and :ref:`services-and-daemons` for more details.

.. _snapcraft-yaml-bus-name:

apps.<app-name>.bus-name
------------------------
*optional*

The bus name that the application or service exposes via D-Bus.

**Type:** ``string``

See :ref:`services-and-daemons` for more information.

.. _snapcraft-yaml-command:

apps.<app-name>.command
-----------------------
The command to run inside the snap when ``<app-name>`` is invoked.

**Type:** ``string``

The command can be in either a snap runtime's command path, ``$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin``, or an executable path relative to ``$SNAP``.

If daemon is set, this will be the command to run the service.

Only a snap with *classic* confinement can use a relative path because the ``PATH`` environment variable isn't modified by a wrapper in classic confinement. See :ref:`classic-confinement` for more details.

**Examples:** ``app-launch`` for an executable placed under ``$SNAP/bin``. With ``classic`` confinement, ``bin/app-launch`` for an executable placed under ``$SNAP/bin``.

**Note:** The command must consist only of alphanumeric characters, spaces, and the following special characters: / . _ # : $ -.  If other characters are required, a wrapper script should be used for the command.

apps.<app-name>.command-chain
-----------------------------
A list of commands to be executed prior to ``apps.<app-name>.command``.

**Type:** ``string``

The list is executed, in order, before running the ``apps.<app-name>.command``.

See `Proposal: support command-chain in apps and hooks <proposal-support-command-chain_>`_ for more details.

To ensure that the Snapd distribution user running supports this feature, insert the ``command-chain`` value to the ``assumes`` property.

apps.<app-name>.common-id
-------------------------
An identifier to a desktop-id within an external appstream file.

**Type:** ``string``

See :ref:`using-external-metadata` for more details.

.. _snapcraft-yaml-daemon:

apps.<app-name>.daemon
----------------------
Declares that ``<app-name>`` is a system daemon.

**Type:** ``enum``

Can be one of the following:

- ``simple``: the command is the main process.
- ``oneshot``: the configured command will exit after completion
- ``forking``: the configured command calls ``fork()`` as part of its start-up. The parent process is then expected to exit when start-up is complete
- ``notify``: the command configured will send a signal to systemd to indicate that it's running.  See :ref:`services-and-daemons` for further details.
- ``dbus``: the command will indicate that it is running when it obtains a bus
  name, either using :ref:`bus-name <snapcraft-yaml-bus-name>` or
  :ref:`activates-on <snapcraft-yaml-activates-on>`.

apps.<app-name>.desktop
-----------------------
Location of the ``.desktop`` file.

**Type:** ``string``

A path relative to the *prime* directory pointing to a desktop file, commonly used to add an application to the launch menu. Snapcraft will take care of the rest.

**Examples:** ``usr/share/applications/my-app.desktop`` and ``share/applications/my-app.desktop``

apps.<app-name>.environment
---------------------------
A set of key-value pairs specifying the contents of environment variables.

**Type:** ``dict``

Key is the environment variable name; Value is the contents of the environment variable.

**Example:** ``LANG: C.UTF-8``

apps.<app-name>.extensions
--------------------------
:ref:`snapcraft-extensions` apply to this application.

**Type:** ``list[string]``

**Example:** ``[gnome-3-28]``

apps.<app-name>.install-mode
----------------------------
Defines whether a freshly installed daemon is started automatically.

**Type:** ``string``

Requires ``daemon`` to be set in ``app`` metadata. Set to _disable_ to defer daemon startup to the snap,  which could then use :ref:`snapctl <using-the-snapctl-tool>` with a :ref:`hook <supported-snap-hooks>`, for instance, or another management agent. Can be one of the following:

``enable`` or ``disable`` (defaults to ``enable``)

apps.<app-name>.plugs
---------------------
Plugs for :ref:`interfaces <interface-management>` to connect to.

**Type:** ``list[string]``

``<app-name>`` will make these plug connections when running in *strict confinement*. For interfaces that need *attributes*, see top-level :ref:`snapcraft-top-level-metadata-plugs`.

**Example:** ``[home, removable-media, raw-usb]``

apps.<app-name>.post-stop-command
---------------------------------
Runs a command from inside the snap after a service stops.

**Type:** ``string``

Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.refresh-mode
----------------------------
Controls whether the daemon should be restarted during a snap refresh.

**Type:** ``string``

Requires ``daemon`` to be set in ``app`` metadata. Can be one of the following:

``endure`` or ``restart`` (defaults to ``restart``)

apps.<app-name>.slots
---------------------
Slots for :ref:`interfaces <interface-management>` to connect to.

**Type:** ``list[string]``

``<app-name>`` will make these slot connections when running in ``strict`` confinement only. For interfaces that need *attributes*, see top-level :ref:`snapcraft-top-level-metadata-slots`.

**Example:** ``[home, removable-media, raw-usb]``

apps.<app-name>.start-timeout
-----------------------------
The length of time to wait for a daemon to start.

**Type:** ``string``

Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. Termination is via ``SIGTERM`` (and ``SIGKILL`` if that doesn't work). 

Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.stop-command
----------------------------
The path to a command inside the snap to run to stop the service.

**Type:** ``string``

Requires ``daemon`` to be set in ``app`` metadata.

apps.<app-name>.stop-timeout
----------------------------
The length of time to wait before terminating a service.

**Type:** ``string``

Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. Termination is via ``SIGTERM`` (and ``SIGKILL`` if that doesn't work).

Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.timer
---------------------
Schedules when, or how often, to run a service or command.

**Type:** ``timer string``

See _timer-string-format for further details on the required syntax.

Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.restart-condition
---------------------------------
Condition to restart the daemon under.

**Type:** ``enum``

Defaults to ``on-failure``. Other values are  ``[on-failure|on-success|on-abnormal|on-abort|always|never]``. Refer to the `systemd.service manual`_ for details.

Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.restart-delay
-----------------------------
The length of time to wait before daemon restarts.

**Type:** ``string``

Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.  Defaults to unset.

See the systemd.service manual on RestartSec_ for details. Requires ``daemon`` to be set in the ``app`` metadata.

apps.<app-name>.sockets
-----------------------
Maps a daemon's sockets to services and activates them.

**Type:** ``dict``

Requires an activated daemon socket.

Requires ``apps.<app-name>.plugs`` to declare the ``network-bind`` plug.

apps.<app-name>.socket-mode
---------------------------
The mode of a socket in *octal*.

**Type:** ``integer``

apps.<app-name>.listen-stream
-----------------------------
The socket abstract name or socket path.

**Type:** ``string``

Sockets should go to a map of ``<socket-name>`` to objects which specify the listen-stream and (optionally) the socket-mode.

TCP socket syntax: ``\<port\>``, ``[::]:\<port\>``, ``[::1]:\<port\>`` and ``127.0.0.1:\<port\>``

UNIX socket syntax: ``$SNAP_DATA/\<path\>``, ``$SNAP_COMMON/<path>`` and ``@snap.\<snap name\>.<suffix>``

apps.<app-name>.passthrough
---------------------------
``<app-name>`` attributes to pass through to ``snap.yaml`` without snapcraft validation.

**Type:** ``type[object]``

See :ref:`using-in-development-features-in-snapcraft-yaml` for further details.

apps.<app-name>.watchdog-timeout
--------------------------------
This value declares the service watchdog timeout.

**Type:** ``string``

Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. For watchdog to work, the application requires access to the _systemd_ notification socket, which can be declared by listing a daemon-notify plug in the plugs section.

Requires ``daemon`` to be set in the ``app`` metadata.

architectures
-------------
*optional*

List of build and run architectures.

**Type:** ``list[object]``

For more details, see :ref:`architectures`.

assumes
-------
*optional*

A list of features that must be supported by the core in order for this snap to install. For example, to make the snap only installable on certain recent version of snapd (like 2.38) you can specify ``snapd2.38`` as an item in this list.

See :ref:`snapcraft-top-level-metadata-assumes` for other potential values.

**Type:** ``list[string]``

base
----
*mandatory*

A snap of type :ref:`base <base-snaps>` to be used as the execution environment for this snap.

**Examples:** ``'core'``, ``'core18'``, ``'core20'``

This is mandatory unless the ``type`` parameter is set to either ``base``, ``kernel``, or ``snapd``.

build-base
----------
*optional*

Used to build a :ref:`base <base-snaps>` snap when the base is unavailable or has yet to be bootstrapped. See :ref:`Building a base snap <building-a-base-snap>` for details.

**Examples:** ``'core20'``, ``'core22'``

Requires that the ``type`` parameter is set to ``base``.

compression
-----------
*optional*

Sets the compression type for the snap.

**Type**: ``string``

Can be ``xz`` or ``lzo`` . Defaults to ``xz`` when not specified. See :ref:`snapcraft-top-level-metadata-compression` for further details.

.. _snapcraft-yaml-confinement:

confinement
-----------
*optional*

Determines if the snap should be restricted in access or not.

**Type:** ``enum`` Possible values are ``strict`` (for no access outside of declared ``interfaces`` through ``plugs``), ``devmode`` (for unrestricted access) or ``classic``. For more information, refer to :ref:`snap-confinement`.

**Examples:** ``strict``, or ``devmode``

contact
-------
*optional*

Contact information for the snap.

**Type:** ``string|list[string]`` Links or email address for users to contact the publisher of the snap.

**Example:** ``contact@product.org``

description
-----------
*mandatory*

Multi-line description of the snap.

**Type:** ``string`` A more in-depth look at what your snap does and who may find it most useful.

donation
--------
*optional*

Donation information for the snap.

**Type:** ``string|list[string]`` Links to provide donations for the publisher of the snap.

**Example:** ``https://patreon.com``

epoch
-----
*optional*

Controls when users receive a configuration-breaking application release.

**Type:** ``integer``

Incrementing the epoch in the new release stops old users automatically refreshing to the new version. See :ref:`snap-epochs` for further details.

grade
-----
*optional*

Defines the quality ``grade`` of the snap.

**Type:** ``enum`` Can be either ``devel`` (i.e. a development version of the snap, so not to be published to the ``stable`` or ``candidate`` channels) or ``stable`` (i.e. a stable release or release candidate, which can be released to all channels)

**Example:** [``stable`` or ``devel``]

hooks
-----
*optional*

This top-level keyword to define a hook with a plug to access more privileges. See :ref:`supported-snap-hooks` for further details.

**Type:** ``list[string]``

icon
----
*optional*

Path to icon image that represents the snap in the snapcraft.io store pages and other graphical store fronts. *Note that the* `desktop menu <https://en.wikipedia.org/wiki/Start_menu>`__ *does not use this icon. It uses the icon in the* ``.desktop`` *file of the application.*

**Type:** ``string`` It is a relative path to a ``.png`` or ``.svg`` file from the source tree root. The `recommended <https://snapcraft.io/docs/restrictions-on-screenshots-and-videos-in-snap-listings24>`__ size is 256x256 pixels. Aspect ratio needs to be 1:1. Image size can vary from 40x40 to 512x512 px and the file size should not be larger than 256 KB.

**Examples:** ``_package_name_.svg``, or ``snap/gui/logo.png``

issues
------
*optional*

Issue tracker or bug reporting location for the snap.

**Type:** ``string|list[string]`` Links or email address for users to report issues to the publisher of the snap.

**Example:** ``https://github.com/org/project/issues, contact@product.org``

layout
------
*optional*

Modify the execution environment of a strictly-confined snap.

**Type:** ``list[dict]``

Layouts are defined as a key-value map, mapping from a ``<target-path>`` to a layout declaration. See :ref:`Using layouts <snap-layouts>` for more details.

**Examples:** ``/var/lib/foo: bind: $SNAP_DATA/var/lib/foo``

license
-------
*optional*

A license for the snap in the form of an SPDX-expression_ for the license. In the legacy Snapcraft syntax (not using the ``base`` key), this key is only available :ref:`through the passthrough key <using-in-development-features-in-snapcraft-yaml>`.
Currently, `only SPDX 2.1 expressions are supported <SPDX-2.1-support_>`_, refer to `snapd/licenses.go <snapd-licenses_>`_ for accepted expressions.

**Type:** ``string``

**Examples:** ``GPL-3.0``, ``MIT``, ``Proprietary``

name
----
*mandatory*

The identifying name of the snap.

**Type:** ``string``

Max length 40 characters. It must start with an ASCII character and can only contain letters in lower case, numbers, and hyphens, and it can’t start or end with a hyphen. The name must be unique if you want to :ref:`publish to the Snap Store <releasing-your-app>`. For help on choosing a name and registering it on the Snap Store, see :ref:`Registering your app name <registering-your-app-name>`.

**Example:** ``my-awesome-app``

package-repositories
--------------------
*optional*

Adds package repositories, including PPA-type and deb-type repositories.

**Type:** ``list[dict]``

See :ref:`snapcraft-package-repositories` for further information.

parts
-----
A set of independent building blocks.

**Type:** ``dict``

These independent building blocks are known as *parts*, and consist of either code or pre-built packages.

parts.<part-name>
-----------------
The name of the part building block.

**Type:** ``dict``

``<part-name>`` represents the specific name of a building block which can be then referenced by the command line tool (i.e. :command:`snapcraft`).

parts.<part-name>.plugin
------------------------
The plugin to drive the build process.

**Type:** ``string``

Every part drives its build through a plugin, this entry declares the plugin that will drive the build process for ``<part-name>``. Refer to :ref:`snapcraft-plugins` for more information on the available plugins and the specific attributes they add to the ``parts.<part-name>.`` namespace.

parts.<part-name>.source
------------------------
A URL or path to a source tree to build.

**Type:** ``string``

This can be a local path or remote, and can refer to a directory tree, a compressed archive or a revision control repository. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

parts.<part-name>.source-type
-----------------------------
Used when the type-of ``source`` entry cannot be detected.

**Type:** ``enum``

Can be one of the following: ``[bzr|deb|git|hg|local|mercurial|rpm|subversion|svn|tar|zip|7z]``

parts.<part-name>.source-checksum
---------------------------------
Used when ``source`` represents a file.

**Type:** ``string``

Takes the syntax ``<algorithm>/<digest>``, where ``<algorithm>`` can be any of: ``md5``, ``sha1``, ``sha224``, ``sha256``, ``sha384``, ``sha512``, ``sha3_256``, ``sha3_384`` or ``sha3_512``. When set, the source is cached for multiple uses in different snapcraft projects.

parts.<part-name>.source-depth
------------------------------
Depth of history for sources using version control.

**Type:** ``integer``

Source repositories under version control are cloned or checked out with full history. Specifying a depth will truncate the history to the specified number of commits.

parts.<part-name>.source-branch
-------------------------------
Work on a specific branch for source repositories under version control.

**Type:** ``string``

parts.<part-name>.source-commit
-------------------------------
Work on a specific commit for source repositories under version control.

**Type:** ``string``

parts.<part-name>.source-tag
----------------------------
Work on a specific tag for source repositories under version control.

**Type:** ``string``

parts.<part-name>.source-subdir
-------------------------------
A path within the ``source`` to set as the working directory when building. The build will *not* be able to access files outside of this location, such as one level up.

**Type:** ``string``

parts.<part-name>.source-submodules
-----------------------------------
Used to configure which submodules to fetch from the source tree.

**Type:** ``dict``

When defined, only listed submodules are fetched. If empty, no submodules are fetched. If ``submodules`` is not defined, all submodules are fetched by default.

parts.<part-name>.after
-----------------------
Ensures that all the parts listed in ``after`` are staged before this part begins its :ref:`lifecycle <parts-lifecycle-steps>`.

**Type:** ``list[string]``

parts.<part-name>.build-environment
-----------------------------------
**Type:** ``list[string]``

A list of environment variable assignments that is applied during the build step, it is exported in order which allows for later values to override (or modify) earlier values. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

parts.<part-name>.build-snaps
-----------------------------
A list of snap names to install that are necessary to build ``<part-name>``.

**Type:** ``list[string]``

If a specific channel is required, the syntax is of the form ``<snap-name>/<channel>``. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

parts.<part-name>.build-packages
--------------------------------
A list of packages required to build a snap.

**Type:** ``list[string]``

Packages are installed using the host's package manager, such as ``apt`` or ``dnf``, and are required for ``<part-name>`` to build correctly. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

**Example:** ``[libssl-dev, libssh-dev, libncursesw5-dev]``

parts.<part-name>.stage-packages
--------------------------------
A list of packages required at runtime by a snap.

**Type:** ``list[string]``

Packages are installed using the host's package manager, such as ``apt`` or ``dnf``, and are required by ``<part-name>`` to run. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

**Example:** ``[python-zope.interface, python-bcrypt]``

parts.<part-name>.stage-snaps
-----------------------------
A list of snaps required at runtime by a snap.

**Type:** ``list[string]``

Snaps are required by ``<part-name>`` to run. They are fetched using ``snap download``, and are unpacked into the snap being built. This entry supports additional syntax, for more information refer to :ref:`snapcraft-advanced-grammar`.

**Example:** ``[hello, black/latest/edge]``

parts.<part-name>.organize
--------------------------
A map of files to rename.

**Type:** ``dict``

In the key/value pair, the key represents the path of a file inside the part and the value represents how the file is going to be staged.

**Example:** ``bin/snapcraftctl: bin/scriptlet-bin/snapcraftctl``

parts.<part-name>.filesets
--------------------------
A key to represent a group of files, or a single file.

See :ref:`snapcraft-filesets` for further details.

.. _snapcraft-yaml-reference-stage:

parts.<part-name>.stage
-----------------------
A list of files from ``<part-name>`` to stage.

**Type:** ``list[string]``

Rules applying to the list here are the same as those of filesets. Referencing of fileset keys is done with a ``$`` prefixing the fileset key, which will expand with the value of such key.

parts.<part-name>.parse-info
----------------------------
Defines the content to adopt when using external metadata.

Type:  ``list[string]``

It is a relative path to a :ref:`supported metadata file <using-external-metadata>` from the part source, build or install directory (:ref:`SNAPCRAFT_PART_SRC, SNAPCRAFT_PART_BUILD, SNAPCRAFT_PART_INSTALL <parts-lifecycle-parts-directories>`).

See :ref:`using-external-metadata` for more details.

.. _snapcraft-yaml-reference-prime:

parts.<part-name>.prime
-----------------------
A list of files from ``<part-name>`` to :ref:`prime <parts-lifecycle-steps>`.

**Type**: ``list[string]``

Rules applying to the list here are the same as those of filesets. Referencing of fileset keys is done with a ``$`` prefixing the fileset key, which will expand with the value of such key.

parts.<part-name>.prepare
-------------------------
*deprecated*

**The release of** :ref:`Snapcraft 3.0 <release-notes-snapcraft-3-0>` **made this key obsolete.**

**Use** :ref:`override-build <snapcraft-yaml-override-build>` **instead.**

Runs a script before the plugin's :ref:`build step <parts-lifecycle-steps>`.

**Type:** ``multiline string``

The script is run before the build step defined for ``parts.<part-name>.plugin`` starts. The working directory is the base build directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``.

A set of :ref:`environment variables <environment-variables>` will be available to the script.

.. _snapcraft-yaml-override-build:

parts.<part-name>.override-build
--------------------------------
Replaces a plugin's default build process with a script.

**Type:** ``multiline string``

The shell script defined here replaces the :ref:`build step <parts-lifecycle-steps>` of the plugin, defined in `parts.<part-name>.plugin`. The working directory is the base build directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``.  A set of :ref:`environment variables <environment-variables>` will be available to the script.

parts.<part-name>.override-prime
--------------------------------
Replaces a plugin's default prime process with a script.

**Type:** ``multiline string``

The shell script defined here replaces the :ref:`prime step <parts-lifecycle-steps>` of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base prime directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``.  A set of :ref:`environment variables <environment-variables>` will be available to the script.

parts.<part-name>.override-pull
-------------------------------
Replaces a plugin's default pull process with a script.

**Type:** ``multiline string``

The shell script defined here replaces the :ref:`pull step <parts-lifecycle-steps>` of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base pull directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`environment variables <environment-variables>` will be available to the script.

parts.<part-name>.override-stage
--------------------------------
Replaces a plugin's default stage process with a script.

**Type:** ``multiline string``

The shell script defined here replaces the :ref:`stage step <parts-lifecycle-steps>` of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base stage directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``.  A set of :ref:`environment variables <environment-variables>` will be available to the script.

parts.<part-name>.build-attributes
----------------------------------
A list of named attributes to modify the behaviour of plugins.

**Type:** ``enum``

For more information, refer to :ref:`snapcraft-parts-metadata`.

passthrough
-----------
*optional*

Attributes to passthrough to ``snap.yaml`` without validation from snapcraft.

**Type:** ``type[object]``

See :ref:`using-in-development-features-in-snapcraft-yaml` for more details.

plugs
-----
*optional*

A set of plugs that the snap asserts.

**Type:** ``dict``

These plugs apply to all ``apps`` and differs from ``apps.<app-name>.plugs`` in that the type is in a ``dict`` rather than a ``list`` format, ``:`` (colon) must be postfixed to the interface name and shouldn't start with ``-`` (dash-space)

plugs.<plug-name>
-----------------
*optional*

A set of attributes for a plug

**Type:** ``dict``

**Example:** ``read`` attribute for the ``home`` interface

plugs.<plug-name>.<attribute-name>
----------------------------------
*optional*

Value of the attribute

**Type:** ``string``

**Example:** ``all`` for ``read`` attribute of the ``home`` interface

slots
-----
*optional*

A set of slots that the snap provides.

**Type:** ``dict``

These slots apply to all the ``apps``

slots.<slot-name>
-----------------
*optional*

A set of attributes of the slot

**Type:** ``dict``

slots.<slot-name>.<attribute-name>
----------------------------------
*optional*

Value of the attribute

**Type:** ``dict``

source-code
-----------
*optional*

Location where the source of the snap can be found.

**Type:** ``string`` Repository link to where the snap project assets can be found.

**Example:** ``https://github.com/org/project.git``

summary
-------
*mandatory*

Sentence summarising the snap.

**Type:** ``string``

Max len. 78 characters, describing the snap in short and simple terms.

**Example:** ``The super cat generator``

title
-----
*optional*

The canonical title of the application, displayed in the software centre graphical frontends.

**Type:** ``string``

Max length 40 characters.

In the legacy Snapcraft syntax (prior to the ``base`` key), this key is only available through the :ref:`passthrough <using-in-development-features-in-snapcraft-yaml>` key.

**Example:** My Awesome Application

type
----
*optional*

The type of snap, implicitly set to ``app`` if not set.

**Type:** ``enum`` For more details, see :ref:`gadget <gadget-snaps>`, :ref:`kernel <the-kernel-snap>` and :ref:`base <base-snaps>`

version
-------
*mandatory*

A user facing version to display.

**Type**: ``string``

Maximum length 32 chars.

Needs to be wrapped with single-quotes when the value will be interpreted by the YAML parser as non-string. This field is mandatory unless version information is provided by ``adopt-info``. See :ref:`using-external-metadata` for details.

**Examples:** ``'1'``, ``'1.2'``, ``'1.2.3'``, ``git`` (will be replaced by a ``git describe`` based version string)

version-script
--------------
:ref:`deprecated <deprecation-notice-10>`

**Deprecated** Use ``snapcraftctl set-version`` :ref:`part scriptlet <meta-scriptlet>` instead. A command to determine the snap’s version string

**Type**: ``string``

Runs from the working directory of the source tree root, and prints a version string to the standard output. Replaces the value of the version keyword. The version keyword is still mandatory (but ignored).

website
-------
*optional*

Publisher website for the snap.

**Type:** ``string`` Product link for the snap.

**Example:** ``https://project.com``


.. _timer-string-format: https://forum.snapcraft.io/t/6562
.. _`systemd.service manual`: https://www.freedesktop.org/software/systemd/man/systemd.service.html#Restart=
.. _RestartSec: https://www.freedesktop.org/software/systemd/man/systemd.service.html#RestartSec=
.. _SPDX-expression: https://spdx.org/licenses/
.. _SPDX-2.1-support: https://github.com/snapcore/snapd/blob/89b5855d44686008f855582bdfd7b2bf7b1a157c/spdx/validate.go#L24
.. _snapd-licenses: https://github.com/snapcore/snapd/blob/master/spdx/licenses.go
.. _proposal-support-command-chain: https://forum.snapcraft.io/t/6112
