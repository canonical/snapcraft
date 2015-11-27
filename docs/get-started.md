# Setting up your Ubuntu development host

Ubuntu is a great and convenient OS for developers. Snappy developer tools are
readily available to enable app developers familiar with Ubuntu to port and
write new software for a snappy-based system easily.

For app developers that want the latest stable tools to work on Snappy
technology, we recommend to use the latest classic Ubuntu Long-Term Support
(LTS) release as the host. At the time of writing this is Ubuntu 14.04 LTS. For
those not using an Ubuntu machine (and you should), you can use a VM
(VirtualBox, VMware, Vagrant) to execute your Ubuntu development host.

Once your Ubuntu host system is up and running, you can then enable the
snappy-tools PPA to get the latest tools to develop for Snappy. A PPA is a
Personal Package Archive that developers can subscribe to install and get
frequent updates of the software the archive contains. Open up a terminal with
`Ctrl+Alt+T` and type the following command to add the snappy-tools PPA to
your system:

	$ sudo apt-add-repository ppa:snappy-dev/tools
	$ sudo apt update

After that, running the following command will install the `snappy-tools`
package, which will in turn install the optimal selection of Snappy development
software to your system.

	$ sudo apt install snappy-tools

The snappy-tools PPA is officially supported by the Snappy Core team for
Ubuntu LTS releases.  In addition to it, we try to keep snappy-tools also
conveniently available for the latest Ubuntu stable release as well as the
current development release for those who prefer those as host. For a
production environment however, we recommend using an Ubuntu LTS-based host.

This is the most important selection of tools you will get after installation:

	snappy build		    - make snap packages out of a file hierarchy
	snappy-remote 	    - run snappy operations on remote snappy target by IP
	snapcraft 		      - the snap build tool for all snaps
	ubuntu-device-flash	- image creation tool for snappy ubuntu

# Next

How about putting together [your first snap](your-first-snap.md) now?
