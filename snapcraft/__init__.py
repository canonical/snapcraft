# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

"""Plugins drive the build process for a part.
Each part can use an individual plugin that understands how to work with
the declared sources.

These plugins have a lifecycle that consists of the following steps:

    - pull
    - build
    - stage
    - strip
    - snap

# Lifecycle

## Pull
This is the first step. This is where content is downloaded, e.g. checkout a
git repository or download a binary component like the Java SDK. Snapcraft
will place the downloaded content for each part in that part's
`parts/<part-name>/src` directory.

## Build
This is the step that follows pull. Each part is built in its
`parts/part-name/build` directory and installs itself into
`parts/part-name/install`.

## Stage
After the build step of each part, the parts are combined into a single
directory tree that is called the "staging area". It can be found
under the `./stage` directory.

This is the area where all parts can share assets such as libraries to link
against.

## Strip
The strip step moves the data into a `./snap` directory. It contains only
the content that will be put into the final snap package, unlike the staging
area which may include some development files not destined for your package.

The Snappy metadata information about your project will also now be placed
in `./snap/meta`.

This `./snap` directory is useful for inspecting what is going into your
snap and to make any final post-processing on snapcraft's output.

## Snap
The final step builds a snap package out of the `snap` directory.

# Common keywords

There are common builtin keywords provided to a snapcraft plugin which can
be used in any part irrespective of the plugin, these are

    - after:
      (list of strings)
      Specifies any parts that should be built before this part is.  This
      is mostly useful when a part needs a library or build tool built by
      another part.
      If a part listed in `after` is not defined locally, it will be
      searched for in the wiki (https://wiki.ubuntu.com/Snappy/Wiki)
    - stage-packages:
      (list of strings)
      A list of Ubuntu packages to use that are needed to support the part
      creation.
    - build-packages:
      (list of strings)
      A list of Ubuntu packages to be installed on the host to aid in
      building the part. These packages will not go into the final snap.
    - organize:
      (yaml subsection)
      A dictionary exposing replacements, the key is the internal filename
      whilst the value is the exposed filename, filesets will refer to the
      exposed named applied after organization is applied.
      This can be used to avoid conflicts by renaming files or using a
      different layout from what came out of the build, e.g.;
      `/usr/local/share/icon.png` -> `/usr/share/icon.png`.
    - filesets:
      (yaml subsection)
      A dictionary with filesets, the key being a recognizable user defined
      string and its value a list of filenames to be included or
      excluded. Globbing is achieved with * for either inclusions or
      exclusion. Exclusions are denoted by an initial `-`.
      Globbing is computed from the part's install directory in
      `parts/<part-name>/install`.
    - stage:
      (list of strings)
      A list of files from a part's installation to expose in `stage`.
      Rules applying to the list here are the same as those of filesets.
      Referencing of fileset keys is done with a $ prefixing the fileset
      key, which will expand with the value of such key.
    - snap:
      (list of strings)
      A list of files from a part's installation to expose in `snap`.
      Rules applying to the list here are the same as those of filesets.
      Referencing of fileset keys is done with a $ prefixing the fileset
      key, which will expand with the value of such key.
"""

from snapcraft._baseplugin import BasePlugin                       # noqa
from snapcraft._options import ProjectOptions                      # noqa
from snapcraft._help import topic_help                             # noqa
from snapcraft._store import login, logout, upload, register_name  # noqa
from snapcraft import common                                       # noqa
from snapcraft import plugins                                      # noqa
from snapcraft import sources                                      # noqa
from snapcraft.internal import repo                                # noqa
