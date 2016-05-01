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
"""

import logging
import os
import platform
import shutil

import snapcraft
from snapcraft import sources

logger = logging.getLogger(__name__)

_NODEJS_BASE = 'node-v{version}-linux-{arch}'
_NODEJS_VERSION = '4.2.2'
_NODEJS_TMPL = 'https://nodejs.org/dist/v{version}/{base}.tar.gz'
_NODEJS_ARCHES = {
    'i686': 'x86',
    'x86_64': 'x64',
    'armv7l': 'armv7l',
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
            'default': [],
        }

        if 'required' in schema:
            del schema['required']

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['build-properties'].append('node-packages')

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self._npm_dir = os.path.join(self.partdir, 'npm')
        self._nodejs_tar = sources.Tar(_get_nodejs_release(), self._npm_dir)

    def pull(self):
        super().pull()
        os.makedirs(os.path.join(self.partdir, 'npm'))
        self._nodejs_tar.download()

    def clean_pull(self):
        super().clean_pull()

        # Remove the npm directory (if any)
        if os.path.exists(self._npm_dir):
            shutil.rmtree(self._npm_dir)

    def build(self):
        super().build()
        self._nodejs_tar.provision(
            self.installdir, clean_target=False, keep_tarball=True)
        for pkg in self.options.node_packages:
            self.run(['npm', 'install', '-g', pkg])
        if os.path.exists(os.path.join(self.builddir, 'package.json')):
            self.run(['npm', 'install', '-g'])


def _get_nodejs_base():
    machine = platform.machine()
    if machine not in _NODEJS_ARCHES:
        raise EnvironmentError('architecture not supported ({})'.format(
            machine))
    return _NODEJS_BASE.format(version=_NODEJS_VERSION,
                               arch=_NODEJS_ARCHES[machine])


def _get_nodejs_release():
    return _NODEJS_TMPL.format(version=_NODEJS_VERSION,
                               base=_get_nodejs_base())
