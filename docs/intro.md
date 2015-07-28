# Intro

Snapcraft allows easy crafting of packages for Snappy Ubuntu. It makes it
easy to incorporate components from different sources like GitHub, Launchpad,
or npm.

# Snappy

Snappy Ubuntu Core is a new rendition of Ubuntu with transactional updates — a
minimal server image with the same libraries as today’s Ubuntu, but
applications are provided through a simpler mechanism.

Snappy apps and Ubuntu Core itself can be upgraded atomically and rolled back
if needed. Apps are also strictly confined and sandboxed to safeguard your
data and system.

# Key concepts

A .snap package for the Ubuntu Core system contains all its
dependencies. This has a couple of advantages over traditional deb or
rpm based dependency handling, the most important being that a
developer can always be assured that there are no regressions triggered by
changes to the system underneath their app.

Snapcraft makes bundling these dependencies easy by allowing you to
specify them as “parts” in the snapcraft.yaml file.

## Parts

A central aspect of a snapcraft recipe is a “part”. A part is a piece
of software or data that the snap package requires to work or to
build other parts. Each part is managed by a snapcraft plugin and parts
are usually independent of each other.

## Plugins

Snapcraft plugins are written in Python and have a yaml
description. A lot of default plugins are included, for example for
projects written in Go, Java, Python or C. It is also possible
to simply download binary content as part of the snapcraft recipe.

## Lifecycle

Each part goes through the following steps:

### Pull

The first is that each part is pulled. This step will download
content, e.g. checkout a git repository or download a binary component
like the Java SDK. Snapcraft will create a parts/ directory with
sub-directories like parts/part-name/src for each part that contains
the downloaded content.

#### Build

The next step is that each part is build in its parts/part-name/build
directory and installs itself into parts/part-name/install.

### Stage

After the build of each part the parts are combined into a single
directory tree that is called the “staging area”. It can be found
under the ./stage directory.

This ./stage directory is useful for building outside code that isn’t in the
snapcraft.yaml recipe against the snap contents. For example, you might build a
local project against the libraries in ./stage by running
`snapcraft shell make`. Though in general, you are encouraged to add even local
projects to snapcraft.yaml with a local `source:` path.

### Snap

The snap step moves the data into a ./snap directory. It contains only
the content that will be put into the final snap package, unlike the staging
area which may include some development files not destined for your package.

The Snappy metadata information about your project will also now be placed in
./snap/meta.

This ./snap directory is useful for inspecting what is going into your snap
and to make any final post-processing on snapcraft’s output.

### Assemble

The final step builds a snap package out of the snap directory. This .snap file
can be uploaded to the Ubuntu Store and published directly to Snappy users.

# Next

After introducing the key concept of snapcraft it is probably a good
time to look at the tutorial in docs/tutorial.md to see how it works
for an example project.
