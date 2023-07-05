.. 7160.md

.. _the-hunspell-dictionaries-content-snaps:

The Hunspell Dictionaries Content Snaps
=======================================

The Hunspell Dictionaries Content Snap is an attempt to allow multiple snaps to share a single copy of the Hunspell spellchecking dictionaries for multiple languages via the content interface. It allows snapped apps. that implements spellchecking via the infamous Hunspell library to not ship independent copies of the spellchecking dictionaries in their snaps, which takes about 200MiB(uncompressed)/32MiB(squashed) of disk space.

This snap does not feature any applications and is not intended to be used by regular users, instead, it is for snap packagers to implement a counterpart plug to connect and map its content into their snaps.

How to Use
----------

The consumer snap’s snapcraft.yaml must define a counterpart plug for connection:

.. code:: yaml

   plugs:
     hunspell-dictionaries:
       interface: content
       default-provider: _provider_snap_name_
       target: $SNAP/usr/share/hunspell

The list of currently supported *provider_snap_name*:

-  `hunspell-dictionaries-1-3-1604 <https://snapcraft.io/hunspell-dictionaries-1-3-1604>`__ (targeting ``core`` base)
-  `hunspell-dictionaries-1-6-1804 <https://snapcraft.io/hunspell-dictionaries-1-6-1804>`__ (targeting ``core18`` base)
-  `hunspell-dictionaries-1-7-2004 <https://snapcraft.io/hunspell-dictionaries-1-7-2004>`__ (targeting ``core20`` base)

For maximum compatibility, pick the one that targets the same base.

Related Links
~~~~~~~~~~~~~

-  Packaging recipe source code https://code.launchpad.net/hunspell-dictionaries-snaps
-  Desktop: allow access to host hunspell dictionaries - snapd - snapcraft.io https://snapcraft.io/docs/desktop-allow-access-to-host-hunspell-dictionaries
-  The content interface documentation <the-content-interface.md>

Support
~~~~~~~

Please refer our project’s issue tracker https://bugs.launchpad.net/hunspell-dictionaries-snaps

or create a new topic under ``snap`` category in the Snapcraft Forums https://forum.snapcraft.io

Join Snapcrafters
~~~~~~~~~~~~~~~~~

<join-snapcrafters.md>
