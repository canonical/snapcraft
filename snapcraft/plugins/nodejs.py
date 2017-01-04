# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""The nodejs plugin is useful for node/npm based parts.

The plugin uses node to install dependencies from `package.json`. It
also sets up binaries defined in `package.json` into the `PATH`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - node-packages:
      (list)
      A list of dependencies to fetch using npm.
    - node-engine:
      (string)
      The version of nodejs you want the snap to run on.
    - npm-run:
      (list)
      A list of targets to `npm run`.
      These targets will be run in order, after `npm install`
"""

import logging
import os
import platform
import shutil

import snapcraft
from snapcraft import sources

logger = logging.getLogger(__name__)

_NODEJS_BASE = 'node-v{version}-linux-{arch}'
_NODEJS_VERSION = '4.4.4'
_NODEJS_TMPL = 'https://nodejs.org/dist/v{version}/{base}.tar.gz'
_NODEJS_ARCHES = {
    'i686': 'x86',
    'x86_64': 'x64',
    'armv7l': 'armv7l',
    'aarch64': 'arm64',
}


class NodePlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['node-packages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': []
        }
        schema['properties']['node-engine'] = {
            'type': 'string',
            'default': _NODEJS_VERSION
        }
        schema['properties']['npm-run'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': False,
            'items': {
                'type': 'string'
            },
            'default': []
        }

        if 'required' in schema:
            del schema['required']

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['node-packages', 'npm-run']

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['node-engine']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self._npm_dir = os.path.join(self.partdir, 'npm')
        self._nodejs_tar = sources.Tar(get_nodejs_release(
            self.options.node_engine), self._npm_dir)

    def pull(self):
        super().pull()
        os.makedirs(self._npm_dir, exist_ok=True)
        self._nodejs_tar.download()
        # do the install in the pull phase to download all dependencies.
        self._npm_install(rootdir=self.sourcedir)

    def clean_pull(self):
        super().clean_pull()

        # Remove the npm directory (if any)
        if os.path.exists(self._npm_dir):
            shutil.rmtree(self._npm_dir)

    def build(self):
        super().build()
        self._npm_install(rootdir=self.builddir)

    def _npm_install(self, rootdir):
        self._nodejs_tar.provision(
            self.installdir, clean_target=False, keep_tarball=True)
        npm_install = ['npm', '--cache-min=Infinity', 'install']
        for pkg in self.options.node_packages:
            self.run(npm_install + ['--global'] + [pkg], cwd=rootdir)
        if os.path.exists(os.path.join(rootdir, 'package.json')):
            self.run(npm_install, cwd=rootdir)
            self.run(npm_install + ['--global'], cwd=rootdir)
        for target in self.options.npm_run:
            self.run(['npm', 'run', target], cwd=rootdir)


def _get_nodejs_base(node_engine):
    machine = platform.machine()
    if machine not in _NODEJS_ARCHES:
        raise EnvironmentError('architecture not supported ({})'.format(
            machine))
    return _NODEJS_BASE.format(version=node_engine,
                               arch=_NODEJS_ARCHES[machine])


def get_nodejs_release(node_engine):
    return _NODEJS_TMPL.format(version=node_engine,
                               base=_get_nodejs_base(node_engine))
