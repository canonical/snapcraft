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

"""The gulp plugin is useful for parts that use gulp tasks.

The plugin uses gulp to drive the build.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - gulp-tasks:
      (list)
      A list of dependencies to fetch using npm.
    - node-engine:
      (string)
      The version of nodejs you want the snap to run on.
"""

import logging
import os
import shutil

import snapcraft
from snapcraft import sources
from snapcraft.plugins import nodejs

logger = logging.getLogger(__name__)


class GulpPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        node_properties = nodejs.NodePlugin.schema()['properties']

        schema['properties']['gulp-tasks'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': [],
        }
        schema['properties']['node-engine'] = node_properties['node-engine']
        schema['required'] = ['gulp-tasks']

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['build-properties'] = ['gulp-tasks']
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['pull-properties'].append('node-engine')

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self._npm_dir = os.path.join(self.partdir, 'npm')
        self._nodejs_tar = sources.Tar(nodejs.get_nodejs_release(
            self.options.node_engine), self._npm_dir)

    def pull(self):
        super().pull()
        os.makedirs(self._npm_dir, exist_ok=True)
        self._nodejs_tar.download()

    def clean_pull(self):
        super().clean_pull()

        # Remove the npm directory (if any)
        if os.path.exists(self._npm_dir):
            shutil.rmtree(self._npm_dir)

    def build(self):
        super().build()

        self._nodejs_tar.provision(
            self._npm_dir, clean_target=False, keep_tarball=True)

        env = os.environ.copy()
        env['PATH'] = '{}:{}'.format(
            os.path.join(self._npm_dir, 'bin'), env['PATH'])
        self.run(['npm', 'install', '-g', 'gulp-cli'], env=env)
        if os.path.exists(os.path.join(self.builddir, 'package.json')):
            self.run(['npm', 'install', '--only-development'], env=env)
        self.run([
            os.path.join(self._npm_dir, 'bin', 'gulp')] +
            self.options.gulp_tasks, env=env)
