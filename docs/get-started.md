# Getting set up

Ubuntu is a great and convenient OS for developers. Snappy developer tools are
readily available to enable app developers familiar with Ubuntu to port and
write new software for a snappy-based system easily.

For app developers that want the latest stable tools to work on Snappy
technology, we recommend to use the latest classic Ubuntu Long-Term Support
(LTS) release as the host. At the time of writing this is Ubuntu 14.04 LTS. For
those not using an Ubuntu machine (and you should), you can use a VM
(VirtualBox, VMware, Vagrant) to execute your Ubuntu development host.

This version of snapcraft only works on Ubuntu 16.04 (Xenial Xerus), for
previous versions of snapcraft, refer to the
[1.x documentation](https://github.com/ubuntu-core/snapcraft/blob/1.x/docs/get-started.md).

Once your Ubuntu host system is up and running, you can then install the
`snappy-tools` package, which will in turn install the optimal selection of
Snappy development software to your system.

	$ sudo apt install snappy-tools

For a production environment, we recommend using an Ubuntu LTS-based host.

This is the most important selection of tools you will get after installation:

	snappy try          - try snaps from a .snap, the [stage] or [snap] dir
	snappy-remote 	    - run snappy operations on remote snappy target by IP
	snapcraft           - the snap build tool for all snaps

# Next

How about putting together [your first snap](your-first-snap.md) now?
