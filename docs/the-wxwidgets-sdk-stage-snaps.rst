.. 10877.md

.. _the-wxwidgets-sdk-stage-snaps:

The wxWidgets SDK Stage Snaps
=============================

   This forum topic is a stub, feel free to complete/improve it.

This stage snap ships built, ready to use wxWidgets libraries and development files to build wxWidgets applications for your snap.

Prerequisites
-------------

-  Currently only ``core18`` base snap is supported, ``core`` base support may be available in the future
-  This stage snap requires you to connect to desktop-gnome-platform content sharing snap (desktop-gtk3 is possible but not supported)

Variants
--------

gtk3
~~~~

Available as the ``wxwidgets-sdk-gtk3`` stage snap.

gtk2 (not implemented yet, contact me if you need one)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use
----------

Merge the following part definition to your Snapcraft recipe:

.. code:: yaml

   parts:
     # The wxWidgets SDK Stage Snaps
     # the-wxwidgets-sdk-stage-snaps.md
     wxwidgets-sdk:
       plugin: nil
       stage-snaps:
       - wxwidgets-sdk-_variant_identifier_
       filesets:
         crash-dialog-support:
         - usr/bin/*addr2line
         library-shared:
         - '**/lib/**/*.so*'
       prime:
       - $crash-dialog-support
       - $library-shared

then make ``wxwidgets-sdk`` a dependency of your consuming part:

::

   parts:
      _consuming_part_name_:
       after:
       - wxwidgets-sdk

Snaps that consumes these stage snaps
-------------------------------------

-  `Lin-Buo-Ren/filezilla-snap: Unofficial Snap Packaging for FileZilla <https://github.com/Lin-Buo-Ren/filezilla-snap>`__ - `patch <https://github.com/Lin-Buo-Ren/filezilla-snap/commit/7b8b8ca84056a3625118bfab307634a44d665468>`__

The implementation
------------------

`Git : Code : wxwidgets-snaps : “Snap Dump” team <https://code.launchpad.net/~snap-dump/wxwidgets-snaps/+git>`__

Support
-------

`Bugs : wxwidgets-snaps <https://bugs.launchpad.net/wxwidgets-snaps>`__
