# Snapcraft: Advanced features

Once you have built [your first snap](your-first-snap.md), you will probably
want to learn more about snapcraft's more advanced features. Having a look at
our selection of examples is a good idea, as we want it to be a good showcase
of what is possible and generally relevant.

## Examples

Our showcase can be found in the actual source of `snapcraft` itself. Check
it out by simply running:

	git clone https://github.com/ubuntu-core/snapcraft
	cd snapcraft/examples

Inspecting the source locally will make easier to build the examples and
play around with them. (You can
[view them online](https://github.com/ubuntu-core/snapcraft/tree/master/examples)
as well.)

### Playing around with the examples

If you just checked out the `snapcraft` source and inspect the examples, you
can start off your explorations by reading the accompanying `snapcraft.yaml`
file and running:

	../../bin/snapcraft

This will inform you of all the steps taken during the creation of the snap.

## Defining your parts

Once you have noted down all the general information about your snap
(like description, vendor information and everything else), naming
the individual parts will define the stucture of your `snapcraft.yaml` file.
Think of parts as individual components of your snap: Where do you pull them
from? How are they built?

### Prerequisites during the build

The example named `downloader-with-wiki-parts` shows how very easy you can
make sure that the relevant build dependencies are installed:

	build-packages: [libssl-dev]

The above will install the `libssl-dev` package from the Ubuntu archive before
an attempted build. If you need a specific version of libssl-dev or a custom
build, you will need to specify a separate part.

Also note that the above will not define which libraries are shipped with the
app. It merely makes sure you have all the relevant build tools installed.


### Pulling and building

If you just intend to pull and build the source, take a look at the `gopaste`
example. In its `snapcraft.yaml` file you can find just this one `parts`
paragraph:

	parts:
 	  gopaste:
    	    plugin: go
            source: git://github.com/wisnij/gopaste/gopasted

It starts off with the name of the specific part (`gopaste` here), the origin
of the part (it's a `git` URL) and how to build it (plugin: `go`).
Other possible scenarios would be Bazaar or Mercurial branches, or local
directories.

### Mixing and matching plugins

An interesting example is `py2-project` because it defines only one part
(`spongeshaker`), but uses two different plugins (`python2` and `make`) to
assemble and build the snap:

	parts:
  	  spongeshaker:
	    plugin: python2
	    source: git://github.com/markokr/spongeshaker.git
	  make-project:
	    plugin: make
	    source: .

The example above mixes and matches parts of different origin. Locally it
provides a binary we intend to ship (the `sha3sum.py` script) and a
`Makefile` (to install our script in the right place).

`spongeshaker` is a python library we will need to pull from git, build as
a python project and bundle along with our script.

A possible use-case for the above would be if you just intend to ship a small
binary which you maintain, but require a library you need to build from git
as opposed to simply including it from the Ubuntu archives.

What's happening during the `snapcraft` run is:

1. Get `spongeshaker` from `git`.
1. Build it as a python project (which will include installing the python
   library in the right place).
1. Running `make` (from the local `Makefile`) and thus installing our
   `sha3sum.py` script in the right place.

### Putting your parts in order

If your app is comprised of multiple parts, it might be necessary to build
and stage parts in a particular order. This can be done by using the `after`
keyword:

	parts:
	    pipelinetest:
	        plugin: make
	        source: lp:~mterry/+junk/pipelinetest
	        after:
	            - libpipeline
	    libpipeline:
	        plugin: autotools
	        source: lp:~mterry/libpipeline/printf


In the case of the `libpipeline` example above, the part named `pipelinetest`
will be built after `libpipeline`. Especially if you need specific
functionality during a build or as part of checks during the `snap` phase,
this will be handy.

### Re-using parts

With snapcraft we want to make it easy to learn from other app vendors and
re-use parts which have worked well for them.

In the `downloader-with-wiki-parts` example, you can see that the `main`
part is built:

	        after:
	            - curl

As we never define the `curl` part in the above example, `snapcraft` will
check the Ubuntu Wiki, which is where we currently host examples of
successful snapcraft parts. The build order in this case would be `curl`,
then `main`.


## Finishing steps

### Individual files

If you are planning to provide binaries and services to the users of your
apps, you need to specify them in your definition first. It's just a matter
of enumerating them.

The `godd` example has one binary:

	binaries:
	  godd:
	    exec: ./bin/godd

The above will take care of making the script executable, adding it to the
user's path and install it in the right place.

For a simple service we can take a look at `gopaste`:

	services:
	  gopaste:
	    description: "gopaste"
	    start: bin/gopasted

You define a name for the service, describe it (so log messages are more
descriptive), declare how to run the service and that's it. For more
thoughts on services and their security, visit the
[snappy policy](https://developer.ubuntu.com/en/snappy/guides/security-policy/).


### Limiting the number of installed files

To check the list of files included in your snap, you can use `dpkg -c` on
the resulting `.snap` file. If you find that certain files should not be
shipped to the user (download size being just one factor), you can
explicitly tell `snapcraft` which files to snap:

	    snap:
	     - usr/lib/x86_64-linux-gnu/libgudev-1.0.so*
	     - usr/lib/x86_64-linux-gnu/libobject-2.0.so*
	     - usr/lib/x86_64-linux-gnu/libglib-2.0.so*
	     - bin/godd*

Here `godd` further defines the list of files to be placed in the app
during the `snap` phase. As you can see above, globs (using asterisks as
wildcard characters) are a good way of handling complexities within the
directory structure.
