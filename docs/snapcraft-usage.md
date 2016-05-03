# Trying snapcraft

In the example directory, you can look at each individual `snapcraft.yaml`
to see how each project is composed. Within each of these directories try
running `snapcraft` to build a snap for each of these or go through the
lifecycle by running:

	$ snapcraft pull
	$ snapcraft build
	$ snapcraft stage
	$ snapcraft strip
	$ snapcraft snap

That sequence of commands basically went through the lifecycle of all the
defined parts. To quickly inspect the parts a Snapcraft project has, open
the `snapcraft.yaml` file of the corresponding example and look at the keys
inside the parts entry.


## Sideloading your snap

Consider the `downloader-with-wiki-parts` example. To install the built snap:

	$ sudo snap install downloader_1.0_amd64.snap

After installing, a summary of installed snaps will be presented, on a vanilla
x86-64 bit system it would look a lot like this:

	Name          Date       Version      Developer
	ubuntu-core   2015-09-17 5            ubuntu
	downloader    2015-10-01 ICIEPfXHQOaC sideload
	generic-amd64 2015-10-01 1.4          canonical

Take notice of the sideload word in the downloader snap, this indicates that
the snap did not come signed from the store, if an app is sideloaded, it
also fakes the version to allow easy iteration without the need to change the
metadata.

## Next

Once you have played around with `snapcraft` for a bit, you might want to take
a look at its [more advanced features] [advanced] or at an overview of the
[snapcraft.yaml syntax] [syntax].


[advanced]: snapcraft-advanced-features.md
[syntax]: snapcraft-syntax.md
