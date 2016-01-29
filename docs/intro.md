# Snapcraft Overview

Snapcraft is a build and packaging tool which helps you package your software
as a snap. It makes it easy to incorporate components from different sources
and build technologies or solutions.

# Key concepts

A `.snap` package for the Ubuntu Core system contains all its
dependencies. This has a couple of advantages over traditional `deb` or
`rpm` based dependency handling, the most important being that a
developer can always be assured that there are no regressions triggered by
changes to the system underneath their app.

Snapcraft makes bundling these dependencies easy by allowing you to
specify them as "parts" in the `snapcraft.yaml` file.

# Snappy

Snappy Ubuntu Core is a new rendition of Ubuntu with transactional updates - a
minimal server image with the same libraries as today's Ubuntu, but
applications are provided through a simpler mechanism.

Snappy apps and Ubuntu Core itself can be upgraded atomically and rolled back
if needed. Apps are also strictly confined and sandboxed to safeguard your
data and system.

## Parts

A central aspect of a snapcraft recipe is a "part". A part is a piece
of software or data that the snap package requires to work or to
build other parts. Each part is managed by a snapcraft plugin and parts
are usually independent of each other.

## Plugin

Each part has a `plugin` associated to it, this `plugin` provides the mechanism
to handle it. Parts are driven through plugins, there are a variety of plugins
already included for python 2 and 3, go, java, and cmake or autotools based
projects.

## Lifecycle

Each part goes through the following steps:

### Pull

The first is that each part is pulled. This step will download
content, e.g. checkout a git repository or download a binary component
like the Java SDK. Snapcraft will create a `parts/` directory with
sub-directories like `parts/part-name/src` for each part that contains
the downloaded content.

### Build

The next step is that each part is built in its `parts/part-name/build`
directory and installs itself into `parts/part-name/install`.

### Stage

After the build of each part the parts are combined into a single
directory tree that is called the "staging area". It can be found
under the `./stage` directory.

This `./stage` directory is useful for building outside code that isn't in the
`snapcraft.yaml` recipe against the snap contents. For example, you might
build a local project against the libraries in `./stage` by running `snapcraft
shell make`. Though in general, you are encouraged to add even local
projects to snapcraft.yaml with a local `source:` path.

For rapid iteration one can run `snappy try` against this directory to have it
mounted in a `snappy` capable system.

### Strip

The strip step moves the data into a `./snap` directory. It contains only
the content that will be put into the final snap package, unlike the staging
area which may include some development files not destined for your package.

The Snappy metadata information about your project will also now be placed in
`./snap/meta`. Snapcraft takes care of generating all the meta-data Snappy
expects. For a breakdown of what this is, have a look at our [Snappy developer
reference](https://developer.ubuntu.com/snappy/guides/packaging-format-apps/).

This `./snap` directory is useful for inspecting what is going into your snap
and to make any final post-processing on snapcraft's output.

For rapid iteration one can run `snappy try` against this directory to have it
mounted in a `snappy` capable system.

### Snap

The final step builds a snap package out of the `snap` directory. This `.snap`
file can be uploaded to the Ubuntu Store and published directly to Snappy
users.

This command can also be used with a directory argument for projects that
are not following the snapcraft lifecycle but which follow the internal
snap format.

# Next

After introducing the key concept of snapcraft it is probably a good
time [get set up](get-started.md) to create your first snap with snapcraft.
