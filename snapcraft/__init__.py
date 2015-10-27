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
    - snap
    - assemble

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

## Snap
The snap step moves the data into a `./snap` directory. It contains only
the content that will be put into the final snap package, unlike the staging
area which may include some development files not destined for your package.

The Snappy metadata information about your project will also now be placed
in `./snap/meta`.

This `./snap` directory is useful for inspecting what is going into your
snap and to make any final post-processing on snapcraft's output.

## Assemble
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
      building the part but not going into the final snap.
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
      A list of files from a part’s installation to expose in `stage`.
      Rules applying to the list here are the same as those of filesets.
      Referencing of fileset keys is done with a $ prefixing the fileset
      key, which will expand with the value of such key.
    - snap:
      (list of strings)
      A list of files from a part’s installation to expose in `snap`.
      Rules applying to the list here are the same as those of filesets.
      Referencing of fileset keys is done with a $ prefixing the fileset
      key, which will expand with the value of such key.
"""

import contextlib
import os

import snapcraft.common
import snapcraft.sources
import snapcraft.repo


class BasePlugin:

    @classmethod
    def schema(cls):
        """Return a json-schema for the plugin's properties as a dictionary.
        Of importance to plugin authors is the 'properties' keyword and
        optionally the 'requires' keyword with a list of required
        'properties'.

        By default the properties will be that of a standard VCS,
        override in custom implementations if required.
        """
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'properties': {
                'source': {
                    'type': 'string',
                },
                'source-type': {
                    'type': 'string',
                    'default': '',
                },
                'source-branch': {
                    'type': 'string',
                    'default': '',
                },
                'source-tag': {
                    'type:': 'string',
                    'default': '',
                },
            },
            'required': [
                'source',
            ]
        }

    @property
    def PLUGIN_STAGE_SOURCES(self):
        """Define additional sources.list."""
        return getattr(self, '_PLUGIN_STAGE_SOURCES', [])

    def __init__(self, name, options):
        self.name = name
        self.build_packages = []
        self.stage_packages = []

        with contextlib.suppress(AttributeError):
            self.stage_packages = options.stage_packages
        with contextlib.suppress(AttributeError):
            self.build_packages = options.build_packages

        self.options = options
        self.partdir = os.path.join(os.getcwd(), "parts", self.name)
        self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
        self.ubuntudir = os.path.join(os.getcwd(), "parts", self.name,
                                      'ubuntu')
        self.installdir = os.path.join(os.getcwd(), "parts", self.name,
                                       "install")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
    def pull(self):
        """Pull the source code and/or internal prereqs to build the part.

        By default, the base implementation for pull will use the following
        part properties to retrieve source code:

        - source
        - source-branch
        - source-tag
        - source-type

        If source is empty or does not exist, the phase will be skipped.

        Override or inherit from this method if you need to implement or
        enhance with custom pull logic.
        """
        if getattr(self.options, 'source', None):
            snapcraft.sources.get(
                self.sourcedir, self.builddir, self.options)

    def build(self):
        """Build the source code retrieved from the pull phase.

        The base implementation does nothing by default. Override this
        method if you need to process the source code to make it runnable.
        """
        pass

    def snap_fileset(self):
        """Return a list of files to include or exclude in the resulting snap

        The staging phase of a plugin's lifecycle may populate many things
        into the staging directory in order to succeed in building a
        project.
        During the stripping phase and in order to have a clean snap, the
        plugin can provide additional logic for stripping build components
        from the final snap and alleviate the part author from doing so for
        repetetive filesets.

        These are the rules to honor when creating such list:

            - includes can be just listed
            - excludes must be preceded by -

        For example::
            (['bin', 'lib', '-include'])
        """
        return ([])

    def env(self, root):
        """Return a list with the execution environment for building.

        Plugins often need special environment variables exported to the
        system for some builds to take place. This is a list of strings
        of the form key=value. The parameter root is the path to this part.

        :param str root: The root for the part
        """
        return []

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if cwd is None:
            cwd = self.builddir
        if True:
            print(' '.join(cmd))
        os.makedirs(cwd, exist_ok=True)
        return snapcraft.common.run(cmd, cwd=cwd, **kwargs)

    def run_output(self, cmd, cwd=None, **kwargs):
        if cwd is None:
            cwd = self.builddir
        if True:
            print(' '.join(cmd))
        os.makedirs(cwd, exist_ok=True)
        return snapcraft.common.run_output(cmd, cwd=cwd, **kwargs)
