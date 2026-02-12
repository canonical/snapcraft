.. _explanation-cryptographic-technology:

Cryptographic technology in Snapcraft
=====================================

Snapcraft uses cryptographic technologies to fetch arbitrary files over the internet,
communicate with local processes, and store credentials. It does not directly implement
its own cryptography, but it does depend on external libraries to do so.

When building snaps, Snapcraft uses different codebases and libraries depending on the
:ref:`base snap <base-snaps>`. This means that the cryptographic technology used also
depends on the base snap.

.. _explanation_cryptography_core24:

core24 and newer
----------------

Snapcraft is built upon Craft Application and derives much of its functionality from
it. In particular, snaps using core24 or newer as their base use Craft Application to
build, so much of Snapcraft's cryptographic functionality is described in the `Craft
Application cryptography`_ documentation.

Downloading build dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:ref:`Plugins <reference-plugins>` use build tools to download and verify build
dependencies. Some plugins can provision their own build tools, while others require the
build tools to be available on the system.

For more information on the use of cryptography for plugins provided by Craft Parts, see
the `Craft Parts cryptography`_ documentation.

For plugins provided by Snapcraft, the following table summarizes how they provision
build tools, and which build tools they use to download and verify dependencies.

.. list-table::
  :header-rows: 1

  * - Plugin
    - Build tools used
    - Method of provisioning the build tools

  * - :ref:`Colcon <reference-colcon-plugin>`
    - ``colcon`` and ``rosdep``
    - Not provisioned

  * - Conda
    - ``conda``
    - Requests library and `curl`_

  * - Flutter
    - ``flutter``
    - Git

  * - :ref:`Matter SDK <reference_matter_sdk_plugin>`
    - ``matter``
    - Git

core22
------

Snaps using core22 as their base snap depend on various external libraries to build.

Public key signing
~~~~~~~~~~~~~~~~~~

Snapcraft supports the adding and verification of arbitrary package repositories. For
more information, see the `Craft Archives cryptography`_ documentation.

The parts system
~~~~~~~~~~~~~~~~

Snapcraft makes use of *parts* in project files for declarative builds. Parts specified
by the user may download arbitrary files, install packages, and modify files in
the build environment. For more information, see the `Craft Parts cryptography`_
documentation.

Creating virtual build environments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft instantiates and executes builds on self-allocated virtual instances. For more
information, see the `Craft Providers cryptography`_ documentation.

Downloading build dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snaps using core22 have the same plugin support as core24. See the :ref:`core24
<explanation_cryptography_core24>` section for more information on how Snapcraft
downloads and verifies build dependencies.

core20
------

Snaps using core20 as their base snap depend on various external libraries to build.

Creating virtual build environments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft instantiates and builds snaps on self-allocated virtual instances. It uses
the `Requests`_ library to install Multipass on Windows. Build environments for other
operating systems are handled by the local `snap daemon (snapd)`_.

Communication with snapd
~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft uses the Requests library to communicate over Unix sockets with snapd. These
requests fetch information about required software. If the software is missing,
Snapcraft will install it through snapd. This is done by querying the `snapd API
<https://snapcraft.io/docs/reference/development/snapd-rest-api/>`__ with URLs built
dynamically and sanitized by `urllib`_.

Sources
~~~~~~~

Downloading repositories
^^^^^^^^^^^^^^^^^^^^^^^^

When a part sources a remote repository, Snapcraft clones the repository with the
appropriate version control tool. The protocol used, such as ``SSH`` or ``HTTPS``,
depends on the source URL and support from the version control tool.

.. list-table::
  :header-rows: 1

  * - Version control system
    - Tool used

  * - `Git`_
    - ``git``

  * - `Bazaar`_
    - ``bzr``

  * - `Mercurial`_
    - ``hg``

  * - `Subversion`_
    - ``svn``

Downloading source files
^^^^^^^^^^^^^^^^^^^^^^^^

When a part sources a ``.deb``, ``.rpm``, ``.snap``, ``.tar``, ``.zip``, ``.7z``, or an
executable file, Snapcraft calls the Requests library to download it.

If the part has the :ref:`source-type <PartSpec.source_type>` key, then the
integrity of the source file will be verified. The checksum is verified using hashlib,
so all `algorithms available to the hashlib library
<https://docs.python.org/3/library/hashlib.html#hashlib.algorithms_available>`_ can be
used.

Dependencies
~~~~~~~~~~~~

Downloading system packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^

System dependencies are downloaded and verified using snapd and `APT`_.

Downloading build dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Plugins use build tools to download and verify build dependencies. Some plugins can
provision their own build tools, while others require the build tools to be available on
the system. The following table summarizes how plugins provision build tools and which
build tools are used to download and verify dependencies.

.. list-table::
  :header-rows: 1

  * - Plugin
    - Build tools used
    - Method of provisioning the build tools

  * - :ref:`Rust <craft_parts_rust_plugin>`
    - `Cargo <https://doc.rust-lang.org/stable/cargo/>`_
    - `rustup <https://rustup.rs>`_

  * - Catkin
    - ``catkin`` and ``rosdep``
    - Not provisioned

  * - :ref:`Colcon <reference-colcon-plugin>`
    - ``colcon`` and ``rosdep``
    - Not provisioned

  * - Conda
    - ``conda``
    - Requests library and curl

  * - Crystal
    - ``shards``
    - `Crystal snap`_

  * - :ref:`Go <craft_parts_go_plugin>`
    - `Go toolchain <https://go.dev/ref/mod>`_
    - Not provisioned

  * - :ref:`Meson <craft_parts_meson_plugin>`
    - `Meson <https://mesonbuild.com>`_
    - Not provisioned

  * - :ref:`NPM <craft_parts_npm_plugin>`
    - `npm <https://www.npmjs.com/>`_
    - Requests library and curl

  * - :ref:`Python <craft_parts_python_plugin>`
    - `pip <https://pip.pypa.io>`_
    - Not provisioned

Public key signing
~~~~~~~~~~~~~~~~~~

Snapcraft uses cryptographic processes to parse public keys and optionally retrieve them
from remote servers. It does not directly implement its own cryptography, but depends on
`GNU Privacy Guard (GPG)`_ to do so.

A declaration of a package repository includes a mandatory ``key-id`` field that
specifies the fingerprint of the repository's public key. This public key can either be
stored locally or automatically fetched by Snapcraft.

If the key file is located as part of the project's assets, Snapcraft uses GPG as
provided by the official Ubuntu archives to ensure that the file matches the declared
fingerprint. If the key file is not present locally, Snapcraft uses GPG in conjunction
with `dirmngr`_ (also from the Ubuntu archives) to fetch the key from the OpenPGP
keyserver at ``keyserver.ubuntu.com``.

In either scenario, Snapcraft then creates an APT data source for the package repository
referencing the identified key. It does not validate that the remote repository is
signed with the key, as APT itself does this as part of its normal operation.


Remote building
---------------

Remote snap builds use Craft Application. The cryptographic functionality used for
remote builds is described in the  `Craft Application cryptography`_ documentation.

Snapcraft's legacy remote builder uses `launchpadlib`_ to interact with the `Launchpad`_
API and trigger remote builds. Login credentials for Launchpad are stored in a plain
text file in the XDG data directory.

Interaction with storefronts
----------------------------

Snapcraft interfaces with the Snap Store and private stores over the internet. Some
store interactions are driven through Craft Store, such as authentication and listing
releases. For more information, see the `Craft Store cryptography`_ documentation.

For interactions that don't use Craft Store, Snapcraft uses cryptographic processes
to send files between devices and endpoints through the internet. It does not directly
implement its own cryptography, but it does depend on external libraries to do so.

Authentication
~~~~~~~~~~~~~~

Snapcraft uses `macaroons`_, as an authentication mechanism, which are processed by the
`macaroonbakery`_ library. This library validates and manages macaroons as returned by
stores and simplifies the inclusion of macaroons in further requests to stores.

Credentials may additionally be stored on-disk using the `keyring`_ library, which
will use the keyring service provided by the host operating system. If the host does
not have a keyring service, they will instead be stored in a plain text file called
:file:`credentials.json` under the application's data storage directory. A warning is
issued to the terminal when this behavior is triggered. This behavior is available to
ease the usage of Snapcraft inside virtual machines and containers, but is generally
discouraged.

Network connectivity
~~~~~~~~~~~~~~~~~~~~

Snapcraft uses `urllib`_ to simplify and harden the parsing of URLs.

Connections over the internet are mediated by the Requests or `httpx`_ libraries.
libraries. These libraries handle cryptographic operations, such as the TLS handshake,
that are standard requirements for modern internet connections. They are configured to
always attempt HTTPS connections first, but have the ability to communicate over HTTP as
a fallback. The Snap Store does not support HTTP, but this capability is retained to aid
with local testing. Between these two libraries, Snapcraft will use whichever of the two
is invoked by the consuming application.

.. _Apt: https://wiki.debian.org/AptCLI
.. _Bazaar: https://launchpad.net/bzr
.. _Craft Application cryptography: https://canonical-craft-application.readthedocs-hosted.com/en/latest/explanation/cryptography/
.. _Craft Archives cryptography: https://canonical-craft-archives.readthedocs-hosted.com/en/latest/explanation/cryptography/
.. _Craft Parts cryptography: https://canonical-craft-parts.readthedocs-hosted.com/en/latest/explanation/cryptography/
.. _Craft Providers cryptography: https://canonical-craft-providers.readthedocs-hosted.com/en/latest/explanation/cryptography/
.. _Craft Store cryptography: https://canonical-craft-store.readthedocs-hosted.com/en/latest/explanation/cryptography/
.. _Crystal snap: https://snapcraft.io/crystal
.. _curl: https://curl.se/
.. _dirmngr: https://manpages.ubuntu.com/manpages/noble/man8/dirmngr.8.html
.. _Git: https://git-scm.com/
.. _GNU Privacy Guard (GPG): https://gnupg.org/
.. _httpx: https://www.python-httpx.org/
.. _keyring: https://pypi.org/project/keyring/
.. _Launchpad: https://launchpad.net
.. _launchpadlib: https://help.launchpad.net/API/launchpadlib
.. _macaroonbakery: https://pypi.org/project/macaroonbakery/
.. _macaroons: https://research.google/pubs/macaroons-cookies-with-contextual-caveats-for-decentralized-authorization-in-the-cloud/
.. _Mercurial: https://www.mercurial-scm.org/
.. _Requests: https://requests.readthedocs.io/
.. _snap daemon (snapd): https://snapcraft.io/docs/installing-snapd
.. _Subversion: https://subversion.apache.org/
.. _urllib: https://docs.python.org/3/library/urllib.html
