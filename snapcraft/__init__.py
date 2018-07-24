# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Snapcraft plugins drive different build systems

Each part has a build system . Most parts are built from source using one of
a range of build systems such as CMake or Scons. Some parts are pre-built
and just copied into place, for example parts that reuse existing binary
packages.

You tell snapcraft which build system it must drive by specifying the
snapcraft plugin for that part. Every part must specify a plugin explicitly
(when you see a part that does not specify a plugin, that's because the
actual part definition is in the cloud, where the plugin is specified!)

These plugins implement a lifecycle over the following steps:

  - pull:   retrieve the source for the part from the specified location
  - build:  drive the build system determined by the choice of plugin
  - stage:  consolidate desirable files from all the parts in one tree
  - prime:  distill down to only the files which will go into the snap
  - snap:   compress the prime tree into the installable snap file

These steps correspond to snapcraft commands. So when you initiate a
'snapcraft pull' you will invoke the respective plugin for each part in
the snap, in sequence, to handle the source pull. Each part will then have a
fully populated parts/<part-name>/src/ directory. Similarly, if you then say
'snapcraft build' you will invoke the plugin responsible for each part in
turn, to build the part.

# Snapcraft Lifecycle

## Pull

In this first step, source material is retrieved from the specified
location, whether that is a URL for a tarball, a local path to a source tree
inside the snap, a revision control reference to checkout, or something
specific to the plugin such as PyPI. The plugin might also download
necessary artifacts, such as the Java SDK, which are not specific to the
particular part but which are needed by the plugin to handle its type of
build system.

All the downloaded content for each part goes into the
`parts/<part-name>/src/` directory, which acts as a cache to prevent
re-fetching content. You can clean that cache out with 'snapcraft clean'.

## Build

Snapcraft calculates an appropriate sequence to build the parts, based on
explicit 'after' references and the order of the parts in the
snapcraft.yaml. Each part is built in the `parts/<part-name>/build`
directory and installed into `parts/<part-name>/install`.

Note the install step - we might actually want to use built artifacts from
one part in the build process of another, so the `parts/<part-name>/install`
directory is useful as a 'working fresh install' of the part.

Between the plugin, the part definition YAML, and the build system of the
part, it is expected that the part can be built and installed in the right
place.

At this point you have a tree under `parts/` with a subdirectory for every
part, and underneath those, separate src, build and install trees for each
part.

## Stage

We now need to start consolidating the important pieces of each part into a
single tree. We do this twice - once in a very sweeping way that will
produce a lot of extraneous materials but is useful for debugging. This is
the 'stage' step of the lifecycle, because we move a lot of the build output
from each part into a consolidated tree under `stage/` which has the
structure of a snap but has way too much extra information.

The important thing about the staging area is that it lets you get all the
shared libraries in one place and lets you find overlapping content in the
parts. You can also try this directory as if it were a snap, and you'll have
all the debugging information in the tree, which is useful for developers.

Each part describes its own staging content - the files that should be
staged. The part will often describe "chunks" of content, called filesets,
so that they can be referred to as a useful set rather than having to call
out individual files.

## Prime

It is useful to have a directory tree which exactly mirrors the structure of
the final snap. This is the `prime/` directory, and the lifecycle includes a
'prime' step which copies only that final, required content from the
`stage/` directory into the `prime/` directory.

So the `prime/` directory contains only the content that will be put into
the final snap, unlike the staging area which may include debug and
development files not destined for your snap.

The snap metadata will also be placed in `./prime/meta` during the prime
step, so this `./prime` directory is useful for inspecting exactly what is
going into your snap or to conduct any final post-processing on snapcraft's
output.

## Snap

The final step in the snapcraft lifecycle builds a snap out of the `prime/`
directory. It will be in the top level directory, alongside snapcraft.yaml,
called <name>-<version>-<arch>.snap


# Standard part definition keywords

There are several builtin keywords which can be used in any part regardless
of the choice of plugin.

  - after: [part, part, part...]

    Snapcraft will make sure that it builds all of the listed parts before
    it tries to build this part. Essentially these listed dependencies for
    this part, useful when the part needs a library or tool built by another
    part.

    If such a dependency part is not defined in this snapcraft.yaml, it must
    be defined in the cloud parts library, and snapcraft will retrieve the
    definition of the part from the cloud. In this way, a shared library of
    parts is available to every snap author - just say 'after' and list the
    parts you want that others have already defined.

  - build-packages: [pkg, pkg, pkg...]

    A list of packages to install on the build host before building
    the part. The files from these packages typically will not go into the
    final snap unless they contain libraries that are direct dependencies of
    binaries within the snap (in which case they'll be discovered via `ldd`),
    or they are explicitly described in stage-packages.

  - stage-packages: YAML list

    A set of packages to be downloaded and unpacked to join the part
    before it's built. Note that these packages are not installed on the host.
    Like the rest of the part, all files from these packages will make it into
    the final snap unless filtered out via the `snap` keyword.

    One may simply specify packages in a flat list, in which case the packages
    will be fetched and unpacked regardless of build environment. In addition,
    a specific grammar made up of sub-lists is supported here that allows one
    to filter stage packages depending on various selectors (e.g. the target
    arch), as well as specify optional packages. The grammar is made up of two
    nestable statements: 'on' and 'try'.

    Let's discuss `on`.

        - on <selector>[,<selector>...]:
          - ...
        - else[ fail]:
          - ...

    The body of the 'on' clause is taken into account if every (AND, not OR)
    selector is true for the target build environment. Currently the only
    selectors supported are target architectures (e.g. amd64).

    If the 'on' clause doesn't match and it's immediately followed by an 'else'
    clause, the 'else' clause must be satisfied. An 'on' clause without an
    'else' clause is considered satisfied even if no selector matched. The
    'else fail' form allows erroring out if an 'on' clause was not matched.

    For example, say you only wanted to stage `foo` if building for amd64 (and
    not stage `foo` if otherwise):

        - on amd64: [foo]

    Building on that, say you wanted to stage `bar` if building on an arch
    other than amd64:

        - on amd64: [foo]
        - else: [bar]

    You can nest these for more complex behaviors:

        - on amd64: [foo]
        - else:
          - on i386: [bar]
          - on armhf: [baz]

    If your project requires a package that is only available on amd64, you can
    fail if you're not building for amd64:

        - on amd64: [foo]
        - else fail

    Now let's discuss `try`:

        - try:
          - ...
        - else:
          - ...

    The body of the 'try' clause is taken into account only when all packages
    contained within it are valid. If not, if it's immediately followed by
    'else' clauses they are tried in order, and one of them must be satisfied.
    A 'try' clause with no 'else' clause is considered satisfied even if it
    contains invalid packages.

    For example, say you wanted to stage `foo`, but it wasn't available for all
    architectures. Assuming your project builds without it, you can make it an
    optional stage package:

        - try: [foo]

    You can also add alternatives:

        - try: [foo]
        - else: [bar]

    Again, you can nest these for more complex behaviors:

        - on amd64: [foo]
        - else:
          - try: [bar]

  - organize: YAML

    Snapcraft will rename files according to this YAML sub-section. The
    content of the 'organize' section consists of old path keys, and their
    new values after the renaming.

    This can be used to avoid conflicts between parts that use the same
    name, or to map content from different parts into a common conventional
    file structure. For example:

      organize:
        usr/oldfilename: usr/newfilename
        usr/local/share/: usr/share/

    The key is the internal part filename, the value is the exposed filename
    that will be used during the staging process. You can rename whole
    subtrees of the part, or just specific files.

    Note that the path is relative (even though it is "usr/local") because
    it refers to content underneath parts/<part-name>/install which is going
    to be mapped into the stage and prime areas.

  - filesets: YAML

    When we map files into the stage and prime areas on the way to putting
    them into the snap, it is convenient to be able to refer to groups of
    files as well as individual files.  Snapcraft lets you name a fileset
    and then use it later for inclusion or exclusion of those files from the
    resulting snap.

    For example, consider man pages of header files.. You might want them
    in, or you might want to leave them out, but you definitely don't want
    to repeatedly have to list all of them either way.

    This section is thus a YAML map of fileset names (the keys) to a list of
    filenames. The list is built up by adding individual files or whole
    subdirectory paths (and all the files under that path) and wildcard
    globs, and then pruning from those paths.

    The wildcard * globs all files in that path. Exclusions are denoted by
    an initial `-`.

    For example you could add usr/local/* then remove usr/local/man/*:

      filesets:
        allbutman: [ usr/local/*, -usr/local/man/* ]
        manpages: [ usr/local/man ]

    Filenames are relative to the part install directory in
    `parts/<part-name>/install`. If you have used 'organize' to rename files
    then the filesets will be built up from the names after organization.

  - stage: YAML file and fileset list

    A list of files from a part install directory to copy into `stage/`.
    Rules applying to the list here are the same as those of filesets.
    Referencing of fileset keys is done with a $ prefixing the fileset key,
    which will expand with the value of such key.

    For example:

      stage:
        - usr/lib/*   # Everything under parts/<part-name>/install/usr/lib
        - -usr/lib/libtest.so   # Excludng libtest.so
        - $manpages             # Including the 'manpages' fileset

  - snap: YAML file and fileset list

    A list of files from a part install directory to copy into `prime/`.
    This section takes exactly the same form as the 'stage' section  but the
    files identified here will go into the ultimate snap (because the
    `prime/` directory reflects the file structure of the snap with no
    extraneous content).

  - prepare: shell script

    If present, the shell script defined here is run before the `build` step
    of the plugin starts. The working directory is the base build
    directory for the given part. The defined script is run with `/bin/sh`.

    For example:

      prepare: |
        cd scripts
        ./bootstrap.sh

  - build: shell script

    If present, the shell script defined here is run instead of the `build`
    step of the plugin. The working directory is the base build directory
    for the given part. The defined script is run with `/bin/sh`.

    For example:

      plugin: make
      build: |
        make project
        make test
        make special-install

  - install: shell script

    If present, the shell script defined here is run after the `build` step
    of the plugin has finished. The working directory is the base build
    directory for the given part. The defined script is run with `/bin/sh`.

    For example:

      install: |
        sed -i 's|/usr/bin|$SNAP/usr/bin|g' my-bin-artifact.sh
        mv my-bin-artifact.sh $SNAPCRAFT_PART_INSTALL/bin/my-bin-build.sh

  - build-attributes: [attribute1, attribute2]

    A list of special attributes that affect the build of this specific part.
    Supported attributes:

      - no-system-libraries:
        Do not automatically copy required libraries from the system to satisfy
        the dependencies of this part. This might be useful if one knows these
        dependencies will be satisfied in other manner, e.g. via content
        sharing from other snaps.

      - no-install:
        Do not run the install target provided by the plugin's build system.

        Supported by: kbuild

      - debug:
        Plugins that support the concept of build types build in Release mode
        by default. Setting the 'debug' attribute requests that they instead
        build in Debug mode.
"""

from collections import OrderedDict  # noqa
import pkg_resources  # noqa
import yaml  # noqa


def _get_version():
    import os as _os

    if _os.environ.get("SNAP_NAME") == "snapcraft":
        return _os.environ["SNAP_VERSION"]
    try:
        return pkg_resources.require("snapcraft")[0].version
    except pkg_resources.DistributionNotFound:
        return "devel"


# Set this early so that the circular imports aren't too painful
__version__ = _get_version()


from snapcraft._baseplugin import BasePlugin  # noqa

# FIXME LP: #1662658
from snapcraft._store import (  # noqa
    create_key,
    close,
    download,
    revisions,
    gated,
    list_keys,
    list_registered,
    login,
    push,
    push_metadata,
    register,
    register_key,
    release,
    sign_build,
    status,
    validate,
)
from snapcraft import common  # noqa
from snapcraft import extractors  # noqa
from snapcraft import plugins  # noqa
from snapcraft import sources  # noqa
from snapcraft import file_utils  # noqa
from snapcraft import shell_utils  # noqa
from snapcraft.internal import repo  # noqa
from snapcraft.project._project_options import ProjectOptions  # noqa


# Setup yaml module globally
# yaml OrderedDict loading and dumping
# from http://stackoverflow.com/a/21048064 Wed Jun 22 16:05:34 UTC 2016
_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG


def dict_representer(dumper, data):
    return dumper.represent_dict(data.items())


def dict_constructor(loader, node):
    # Necessary in order to make yaml merge tags work
    loader.flatten_mapping(node)
    return OrderedDict(loader.construct_pairs(node))


def str_presenter(dumper, data):
    if len(data.splitlines()) > 1:  # check for multiline string
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


yaml.add_representer(str, str_presenter)
yaml.SafeDumper.add_representer(str, str_presenter)
yaml.add_representer(OrderedDict, dict_representer)
yaml.SafeDumper.add_representer(OrderedDict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)
yaml.SafeLoader.add_constructor(_mapping_tag, dict_constructor)
