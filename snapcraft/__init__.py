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

import contextlib
import logging
import os

import snapcraft.common
import snapcraft.sources
import snapcraft.repo


logger = logging.getLogger(__name__)


class BasePlugin:

    @classmethod
    def schema(cls):
        """Return a json-schema for the plugin's properties as a dictionary.
        Of importance to plugin authors is the 'properties' keyword and
        optionally the 'requires' keyword with a list of required
        'properties'.

        By default the the properties will be that of a standard VCS, override
        in custom implementations if required.
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
        """Implement the pull phase of a plugin's lifecycle.

        The pull phase is the phase in the lifecycle when source code and
        internal requirements for the part to be able to build are taken care
        of.

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
        if not getattr(self.options, 'source', None):
            return True
        try:
            return snapcraft.sources.get(
                self.sourcedir, self.builddir, self.options)
        except ValueError as e:
            logger.error('Unrecognized source %r for part %r: %s.',
                         self.options.source, self.name, e)
            snapcraft.common.fatal()
        except snapcraft.sources.IncompatibleOptionsError as e:
            logger.error(
                'Issues while setting up sources for part \'%s\': %s.',
                self.name,
                e.message)
            snapcraft.common.fatal()

    def build(self):
        """Implement the build phase of a plugin's lifecycle.

        This build phase is the phase in the lifecycle where source code
        retrieved from the pull phase is built using the build mechanism
        the plugin is providing. This is where the plugin generally adds
        value as a plugin.

        The base implementation does nothing by default.
        """
        return True

    def snap_fileset(self):
        """Return a list of files to include or exclude in the resulting snap

        The staging phase of a plugins lifecycle may populate many things
        into the staging directory in order to succeed in building a project.
        During the stripping phase and in order to have a clean snap, the
        plugin can provide additional logic for stripping build components
        from the final snap and alleviate the part author from doing so for
        repetetive filesets.

        These are the rules to honor when creating such list:

            - includes can be just listed
            - excludes must be preceded by -

        For example: (['bin', 'lib', '-include'])
        """
        return ([])

    def env(self, root):
        """Expand a list with the execution environment for building.

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
