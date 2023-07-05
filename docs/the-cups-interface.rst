.. 29873.md

.. _the-cups-interface:

The “cups” Interface
====================

**Introduction**

The ``cups`` interface should be plugged by all Snaps of software which prints, like LibreOffice, Chrome/Chromium, Firefox, Thunderbird, DarkTable, RawTherapee, … It allows them to print via CUPS but it blocks any administrative CUPS operations (like creating or modifying print queues or accessing other user’s jobs).

Before the ``cups`` interface was introduced, you had to plug the ``cups-control`` interface for this, but as this interface does not only allow to list the available printers and their options and send print jobs but also create and modify queues and their permissions, read and delete anyone’s print jobs, … you either had to require the user to manually connect the interface or get explicit permission from the Snap Store team to auto-connect the interface.

As we at `OpenPrinting <http://www.openprinting.org/>`__ want that printing “just works” for your users, this is a bad situation.

Therefore we have created the ``cups`` interface which allows the Snap which plugs it **only** to list printers and the printer’s options, and to print (non-administrative CUPS tasks) **but NOT** to create and modify queues, or delete anyone’s jobs (administrative CUPS tasks). Due to this **the interface gets connected automatically** when your ``cups``-plugging Snap gets loaded from the Snap Store, no special permissions required, no questions asked! Printing will “just work”.

**What do you have to do?** - If you are snapping a **new application** and it has print functionality, just plug ``cups`` as described below. - If you are **maintaining an application Snap** and for just printing you have it plugging ``cups-control``, switch to plugging ``cups`` as described below. - If you are creating or maintaining a **Snap of a printer setup tool**, you have to plug ``cups-control`` and live with the alternatives of manual connection or special permission of the Snap Store team.

**How do I use it?**

First, the new interface got added in **snapd 2.55.3**, the first stable release in the 2.55 series. This version is included in Ubuntu 22.04 LTS (Jammy Jellyfish) but also in the stable channel of the Snap Store, so probably it already landed on your system as an auto update.

Snapping an application with print functionality is easy, do not get turned away by the rather complicated inner workings. Especially your Snap will always talk with the CUPS Snap for all things printing, never with the system’s cupsd (it auto-installs the CUPS Snap as a dependency). This is for security. If the user has a conventional CUPS installation, the CUPS Snap works as a firewall only passing through the non-administrative requests and blocking the administrative ones and if the user uses the CUPS Snap as standard printing environment the CUPS Snap manages this by itself (see my `explanation of the inner workings <https://snapcraft.io/docs/new-interface-cups-for-all-snaps-which-print2?u=till.kamppeter>`__).

First, you start your snapping the same way as you are used to for applications without print functionality. Read about this part elsewhere.

Second, in ``snapcraft.yaml`` let each app with print functionality plug the ``cups`` interface:

::

   apps:
     my-printing-application:
       command: my-printing-app
       plugs:
         - cups
         - network

or

::

   apps:
     lp:
       command: usr/bin/lp
       plugs: [network, home, cups]
     lpstat:
       command: usr/bin/lpstat
       plugs: [network, avahi-observe, cups]

Third, add this line to ``snapcraft.yaml``:

::

   # We need snapd 2.55 or later to have the needed support for the
   # `cups` interface in snapd
   assumes: [snapd2.55]

Do not put this into any section, put it right after the header part for example. Simply copy and paste this blob of lines as you see it here.

These lines make your Snap require at least version 2.55 of snapd, as older versions do not have the ``cups`` interface. I assume that this triggers an auto-update of snapd if needed.

Forth, add the placeholder content interface to trigger the auto-installation of the CUPS Snap. Simply add the following lines to ``snapcraft.yaml``:

::

   # this is not used or needed for anything other than to trigger automatic
   # installation of the cups snap via "default-provider: cups"
   plugs:
     foo-install-cups:
       interface: content
       content: foo
       default-provider: cups
       target: $SNAP_DATA/foo

Do not put this into any section, too, put it again right after the header part for example. Simply copy and paste the whole blob of lines as you see it here. Nothing needs to get adapted to your particular Snap, nor you have to create any directories in your Snap for that to work.

Note: Having to add these lines is only a temporary workaround. The ``default-provider`` functionality is planned to be added to the ``cups`` interface in snapd in the coming months. When this has taken place, all what is needed to use the ``cups`` interface is plugging ``cups`` as described in the second step here and the ``assumes: [snapd2.55]`` line of the third step.

Fifth, build your Snap as you are used to, test it, and upload it into the Snap Store using your habitual method. If a user installs it, the CUPS Snap gets auto-installed and the ``cups`` interface auto-connected and printing out of the Snap “just works”.

DO NOT plug both ``cups`` and ``cups-control`` in the same Snap. This can mess up things, especially as CUPS’ Snap mediation (mechanism to block administrative tasks) works “per-Snap” and not “per-application-in-the-Snap”, meaning that if one of the apps in the Snap plugs ``cups-control`` CUPS assumes the whole Snap plugging ``cups-control`` and allows all apps in the Snap to do administrative operations.

See a `complete example here <https://github.com/snapcore/test-snapd-cups-consumer>`__.

Instructions originally posted in the `cups interface launch thread <https://forum.snapcraft.io/t/new-interface-cups-for-all-snaps-which-print/>`__ and earlier in the `OpenPrinting March 2022 News Post <https://openprinting.github.io/OpenPrinting-News-March-2022/#cups-snap-and-snapd-printing-interface>`__.

Design and development work discussed `here <https://forum.snapcraft.io/t/handling-of-the-cups-plug-by-snapd-especially-auto-connection/>`__ and `here <https://forum.snapcraft.io/t/cups-interface-merged-into-snapd-additional-steps-to-complete/>`__.
