# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import contextlib
import glob
import os
import shutil

from snapcraft.internal import common, sources


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
            'additionalProperties': False,
            'properties': {
                'source': {
                    'type': 'string',
                },
                'source-type': {
                    'type': 'string',
                    'default': '',
                },
                'source-depth': {
                    'type': 'integer',
                    'default': 0,
                },
                'source-branch': {
                    'type': 'string',
                    'default': '',
                },
                'source-commit': {
                    'type': 'string',
                    'default': '',
                },
                'source-tag': {
                    'type:': 'string',
                    'default': '',
                },
                'source-subdir': {
                    'type': 'string',
                    'default': None,
                },
                'disable-parallel': {
                    'type': 'boolean',
                    'default': False,
                },
            },
            'required': [
                'source',
            ],
            'pull-properties': ['source', 'source-type', 'source-branch',
                                'source-commit', 'source-tag',
                                'source-subdir'],
            'build-properties': ['disable-parallel']
        }

    @property
    def PLUGIN_STAGE_SOURCES(self):
        """Define alternative sources.list."""
        return getattr(self, '_PLUGIN_STAGE_SOURCES', [])

    def __init__(self, name, options, project=None):
        self.name = name
        self.build_packages = []
        self.stage_packages = []

        with contextlib.suppress(AttributeError):
            self.stage_packages = options.stage_packages.copy()
        with contextlib.suppress(AttributeError):
            self.build_packages = options.build_packages.copy()

        self.project = project
        self.options = options

        # The remote parts can have a '/' in them to separate the main project
        # part with the subparts. This is rather unfortunate as it affects the
        # the layout of parts inside the parts directory causing collisions
        # between the main project part and its subparts.
        part_dir = name.replace('/', '\N{BIG SOLIDUS}')
        if project:
            self.partdir = os.path.join(project.parts_dir, part_dir)
        else:
            self.partdir = os.path.join(os.getcwd(), 'parts', part_dir)

        self.sourcedir = os.path.join(self.partdir, 'src')
        self.installdir = os.path.join(self.partdir, 'install')

        self.build_basedir = os.path.join(self.partdir, 'build')
        source_subdir = getattr(self.options, 'source_subdir', None)
        if source_subdir:
            self.builddir = os.path.join(self.build_basedir, source_subdir)
        else:
            self.builddir = self.build_basedir

    # The API
    def pull(self):
        """Pull the source code and/or internal prereqs to build the part.

        By default, the base implementation for pull will use the following
        part properties to retrieve source code:

        - source
        - source-branch
        - source-commit
        - source-tag
        - source-type

        If source is empty or does not exist, the phase will be skipped.

        Override or inherit from this method if you need to implement or
        enhance with custom pull logic.
        """
        if getattr(self.options, 'source', None):
            sources.get(self.sourcedir, self.build_basedir, self.options)

    def clean_pull(self):
        """Clean the pulled source for this part.

        The base implementation simply removes the sourcedir. Override this
        method if your pull process was more involved and needs more cleaning.
        """

        if os.path.exists(self.sourcedir):
            if os.path.islink(self.sourcedir):
                os.remove(self.sourcedir)
            else:
                shutil.rmtree(self.sourcedir)

    def build(self):
        """Build the source code retrieved from the pull phase.

        The base implementation only copies sourcedir to build_basedir.
        Override this method if you need to process the source code to make it
        runnable.
        """

        if os.path.exists(self.build_basedir):
            shutil.rmtree(self.build_basedir)

        # FIXME: It's not necessary to ignore here anymore since it's now done
        # in the Local source. However, it's left here so that it continues to
        # work on old snapcraft trees that still have src symlinks.
        def ignore(directory, files):
            if directory is self.sourcedir:
                snaps = glob.glob(os.path.join(directory, '*.snap'))
                if snaps:
                    snaps = [os.path.basename(s) for s in snaps]
                    return common.SNAPCRAFT_FILES + snaps
                else:
                    return common.SNAPCRAFT_FILES
            else:
                return []

        shutil.copytree(
            self.sourcedir, self.build_basedir, symlinks=True, ignore=ignore)

    def clean_build(self):
        """Clean the artifacts that resulted from building this part.

        The base implementation simply removes the build_basedir and
        installdir. Override this method if your build process was more
        involved and needs more cleaning.
        """

        if os.path.exists(self.build_basedir):
            shutil.rmtree(self.build_basedir)

        if os.path.exists(self.installdir):
            shutil.rmtree(self.installdir)

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

    def enable_cross_compilation(self):
        """Enable cross compilation for the plugin."""
        raise NotImplementedError(
            'Building for a different target architecture requires '
            'a plugin specific implementation in the '
            '{!r} plugin'.format(self.name))

    @property
    def parallel_build_count(self):
        """Number of CPU's to use for building.

        Number comes from `project.parallel_build_count` unless the part
        has defined `disable-parallel` as `True`.
        """
        if getattr(self.options, 'disable_parallel', False):
            return 1
        else:
            return self.project.parallel_build_count

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if not cwd:
            cwd = self.builddir
        print(' '.join(cmd))
        os.makedirs(cwd, exist_ok=True)
        return common.run(cmd, cwd=cwd, **kwargs)

    def run_output(self, cmd, cwd=None, **kwargs):
        if not cwd:
            cwd = self.builddir
        os.makedirs(cwd, exist_ok=True)
        return common.run_output(cmd, cwd=cwd, **kwargs)
