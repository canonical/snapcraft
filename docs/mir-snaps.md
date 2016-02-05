# Mir snaps

## Introduction

This document is a collection of current instructions of how to build 2 snaps
as an example for using Mir in the days just after `snapcraft` 2.0 being
released. The first snap contains the `mir-server`, which is the basis for what
Canonical will eventually publish in the store and support for a given Ubuntu
Core release. Which means, until 16.04 becomes a stable release the Mir
version used may change, so do not plan on building any `mir-client` snaps
until the Mir version is frozen along with the released Ubuntu archive. The
second snap is strictly an example of a `mir-client-app`, which developers can
use as a template to create their own. If you are experimenting and simply
building both snaps at the same time you should experience no problems,
however once the `mir-server` snap is in the store, it is important that the
mir-client use the `libmirclient` version corresponding to the `libmirserver`
within their snap.

The `mir-snap` is only targeted for Ubuntu Core. It is intended for
kiosk-like products and applications. If products require many separate UI
applications, then they need Ubuntu Personal.

This document targets Xenial Ubuntu Core and assumes your host machine is
running Xenial (up to date) and hence using Snapcraft 2.0.

> *NOTE: 2016-02-03*
> - snapcraft 2.1 came out today and requires changes. Will followup with
>   updates.
> - also, the most recent Xenial kernel has [a bug creating issues for
>   mir](https://bugs.launchpad.net/ubuntu/+source/linux/+bug/1540731)

## Instructions

You need to be on a Xenial host to obtain the proper tools and the latest
versions of stage packages. Note, one could also use `lxc` for building snaps
in a clean environment. Also, make sure Virtual Machine Manager is installed,
from Ubuntu Software Center or

```
apt-get install virt-manager
```

### Get the Mir Snap running first

If you haven't already, [install the snapcraft tools](get-started.md).

Now grab the `mir-server` snap branch to build.
```
$ bzr branch lp:~mir-team/+junk/mir-server-snap/
$ cd mir-server-snap
$ snapcraft snap
```
Now to create an image off the rolling edge of ubuntu core development. You
will need to do this to get the latest security capabilities for
`display-server` and `mir-client`, do not use an older image.

Also, you will need to use a "special" `ubuntu-device-flash` which is not
quite released to xenial yet, download it from here
https://people.canonical.com/~mvo/all-snaps/

```
$ sudo /dir/to/your/downloaded/ubuntu-device-flash --verbose core rolling --channel edge -o xenial_core_amd64.img --developer-mode --gadget=canonical-pc.canonical --kernel=canonical-pc-linux.canonical --os=ubuntu-core.canonical --size=5
```

Now launch your Virtual Machine Manager application (can be found & installed
from the Ubuntu Software Center). Select the icon for "New Virtual Machine"
(or menu to "File", "New Virtual Machine"). In the dialog select the radio
button for "Import Existing Disk Image", browse to your `*.img` and select
it. You can leave the other defaults. Once you select your way forward, it
should launch another window with your Snappy VM in it. Once the system
settles it will provide you a prompt. Just click on the window to interact
with the VM, to escape select "control+alt" keys. To login, the Snappy image
user is "ubuntu" and password is "ubuntu". Check your ip with ipconfig.

![VM image][vm-image]

Copy your snap over to your running core image & install.
```
$ scp *.snap ubuntu@x.x.x.x:/home/ubuntu
ubuntu# sudo snappy install mir*.snap --allow-unauthenticated
```

The `mir-server` should launch, resulting in a black screen with a mouse.

![Core image][core-image]

You may stop the `mir-server` if you want or have need by `Ctl+delete` or
```
ubuntu# sudo snappy service stop mir
```

Likewise, you may restart by
```
ubuntu# sudo snappy service start mir
```

## Get the Mir-Client snap Running

This section assumes the Mir server is up and running, and you've followed
all of the setup steps from the previous section. This branch is strictly an
example of running a `mir-client` snap on top of a `mir-server` snap.

```
$ bzr branch lp:~mir-team/+junk/snapcraft-mir-client
$ cd snapcraft-mir-client
$ snapcraft snap
```

Copy your snap over to your running core image and install.
```
$ scp *.snap ubuntu@x.x.x.x:/home/ubuntu
```

Quickly double-check Mir server is running, if not just start the service.
```
ubuntu# sudo snappy install mir-client*.snap --allow-unauthenticated
```

At this point you should see some Qml clocks.

![QML clocks][clock-image]

> *Note: 2016-01-26*
>  - This snap is currently unconfined. Working with the security team to
>    rely on the mir-client capability of the system.

#Debugging Snaps

Check or tail `/var/log/syslog` if something isn't launching or running as
expected.

If you run out of memory from loading too many snaps, you can also grow your
image size. For example:
```
$ qemu-img resize xenial_core_amd64.img +1G
```

[vm-image]: https://raw.githubusercontent.com/ubuntu-core/snapcraft/master/docs/images/ubuntucore_in_vmm.png
[core-image]: https://raw.githubusercontent.com/ubuntu-core/snapcraft/master/docs/images/core_running_mir.png
[clock-image]: https://raw.githubusercontent.com/ubuntu-core/snapcraft/master/docs/images/clocks_on_mir_on_ubuntucore.png
