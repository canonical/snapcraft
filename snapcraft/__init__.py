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

import logging
import os

import snapcraft.common
import snapcraft.sources
import snapcraft.repo


logger = logging.getLogger(__name__)


class BasePlugin:

    @property
    def PLUGIN_STAGE_PACKAGES(self):
        return getattr(self, '_PLUGIN_STAGE_PACKAGES', [])

    def __init__(self, name, options):
        self.name = name
        self.options = options
        self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
        self.ubuntudir = os.path.join(os.getcwd(), "parts", self.name, 'ubuntu')
        self.installdir = os.path.join(os.getcwd(), "parts", self.name, "install")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
    def pull(self):
        return True

    def build(self):
        return True

    def snap_fileset(self):
        """Returns one iteratables of globs specific to the plugin:
            - includes can be just listed
            - excludes must be preceded by -
           For example: (['bin', 'lib', '-include'])"""
        return ([])

    def env(self, root):
        return []

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if cwd is None:
            cwd = self.builddir
        if True:
            print(' '.join(cmd))
        return snapcraft.common.run(cmd, cwd=cwd, **kwargs)

    def isurl(self, url):
        return snapcraft.common.isurl(url)

    def get_source(self, source, source_type=None, source_tag=None, source_branch=None):
        try:
            handler_class = _get_source_handler(source_type, source)
        except ValueError:
            logger.error("Unrecognized source '%s' for part '%s'.", source, self.name)
            snapcraft.common.fatal()

        try:
            handler = handler_class(source, self.sourcedir, source_tag, source_branch)
        except snapcraft.sources.IncompatibleOptionsError as e:
            logger.error('Issues while setting up sources for part \'%s\': %s.', self.name, e.message)
            snapcraft.common.fatal()
        if not handler.pull():
            return False
        return handler.provision(self.builddir)

    def handle_source_options(self):
        stype = getattr(self.options, 'source_type', None)
        stag = getattr(self.options, 'source_tag', None)
        sbranch = getattr(self.options, 'source_branch', None)
        return self.get_source(self.options.source,
                               source_type=stype,
                               source_tag=stag,
                               source_branch=sbranch)

    def makedirs(self, d):
        os.makedirs(d, exist_ok=True)

    def setup_stage_packages(self):
        ubuntu = snapcraft.repo.Ubuntu(self.ubuntudir)
        part_stage_packages = getattr(self.options, 'stage_packages', [])
        if self.PLUGIN_STAGE_PACKAGES or part_stage_packages:
            ubuntu.get(self.PLUGIN_STAGE_PACKAGES + part_stage_packages)
            ubuntu.unpack(self.installdir)


def _get_source_handler(source_type, source):
    if source_type is None:
        source_type = _get_source_type_from_uri(source)

    if source_type == 'bzr':
        handler = snapcraft.sources.Bazaar
    elif source_type == 'git':
        handler = snapcraft.sources.Git
    elif source_type == 'mercurial' or source_type == 'hg':
        handler = snapcraft.sources.Mercurial
    elif source_type == 'tar':
        handler = snapcraft.sources.Tar
    else:
        handler = snapcraft.sources.Local

    return handler


def _get_source_type_from_uri(source):
    source_type = ''
    if source.startswith("bzr:") or source.startswith("lp:"):
        source_type = 'bzr'
    elif source.startswith("git:"):
        source_type = 'git'
    elif snapcraft.common.isurl(source):
        raise ValueError()

    return source_type
