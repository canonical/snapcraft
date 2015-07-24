# Into

Snapcraft allows easy crafting of snap packages for the Ubuntu Core
transactional system. It is designed to make it easy to incorporate
components from different sources like github, launchpad or npm.

# Key concepts

All dependencies are bundled in a snap...

## Parts

A central aspect of a snapcraft recipe is a "part". A part is a piece
of software or data that the snap package requires to work or to
build other parts. Each part is managed by a snapcraft plugin and parts
are independent of each other.

## Plugins

Snapcraft plugins are written in python and have a yaml
description. This allows to easily extend snapcraft with custom
plugins. A lot of default plugins are included, for example for
projects written in go, java, python or autotools. It is also possible
to simply download content as part of the snapcraft recipe.

## Lifecycle

Each part goes through the following steps:

### Pull

The first is that each part is pulled. This step will download
content, e.g. checkout a git repository or download a binary component
like the java sdk. Snapcraft will create a parts/ directory with
sub-directories like parts/part-name/src for each part that contains
the downloaded content.

#### Build

The next step is that each part is build in its parts/part-name/build
directory and installs itself into parts/part-name/install.

### Stage

After the build of each part the parts are combined into a single
directory tree that is called the "staging area". It can be found
under the ./stage directory.

### Snap

The snap step move the data into a ./snap directory. It contains only
the content that will be put into the final snap package.

### Assemble

The final step builds a snap package out of the snap directory.

More details on the flow can be found in the docs/flow.md document.

# Next

After introducing the key concept of snapcraft it is probably a good
time to look at the tutorial in docs/tutorial.md to see how it works
for an example project.