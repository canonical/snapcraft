# Trying snapcraft

In the example directory, you can look at each individual `snapcraft.yaml`
to see how each project is composed. Within each of these directories try
running `snapcraft` to build a snap for each of these or go through the
lifecycle by running:

	$ snapcraft pull
	$ snapcraft build
	$ snapcraft stage
	$ snapcraft prime
	$ snapcraft snap

That sequence of commands basically went through the lifecycle of all the
defined parts. To quickly inspect the parts a Snapcraft project has, open
the `snapcraft.yaml` file of the corresponding example and look at the keys
inside the parts entry.


## Sideloading your snap

Consider the `downloader-with-wiki-parts` example. To install the built snap:

	$ sudo snap install downloader_1.0_amd64.snap --dangerous

After installing, a summary of installed snaps will be presented, on a vanilla
x86-64 bit system it would look a lot like this:

       Name                 Version  Rev  Developer  Notes
       core                 16.04.1  888  canonical  -
       downloader           1.0      x1              -
       hello                1.0      x1              -

Take notice of the Rev word in the downloader snap, "x1" indicates that
the snap did not come signed from the store.

## Next

Once you have played around with `snapcraft` for a bit, you might want to take
a look at its [more advanced features] [advanced] or at an overview of the
[snapcraft.yaml syntax] [syntax].


[advanced]: snapcraft-advanced-features.md
[syntax]: snapcraft-syntax.md
