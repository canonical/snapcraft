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

"""This plugin is useful for building parts that use maven.

The maven build system is commonly used to build Java projects.
The plugin requires a pom.xml in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - maven-options:
      (list of strings)
      flags to pass to the build using the maven semantics for parameters.
"""

import glob
import logging
import os
import shutil
from urllib.parse import urlparse

import snapcraft
import snapcraft.common
import snapcraft.plugins.jdk


logger = logging.getLogger(__name__)


_MVN_SETTINGS_FORMAT = (
    '<settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"\n'
    '          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n'
    '          xsi:schemaLocation="http://maven.apache.org/SETTINGS/'
    '1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">\n'
    '  <interactiveMode>false</interactiveMode>\n'
    '  <proxies>\n'
    '    <proxy>\n'
    '      <id>proxy</id>\n'
    '      <active>true</active>\n'
    '      <protocol>http</protocol>\n'
    '      <host>{}</host>\n'
    '      <port>{}</port>\n'
    '      <nonProxyHosts>{}</nonProxyHosts>\n'
    '    </proxy>\n'
    '  </proxies>\n'
    '</settings>\n'
)


class MavenPlugin(snapcraft.plugins.jdk.JdkPlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['maven-options'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        schema['properties']['maven-targets'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [''],
        }

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['build-properties'].append('maven-options')
        schema['build-properties'].append('maven-targets')

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('maven')

    def _use_proxy(self):
        return all([k in os.environ for k in
                    ('SNAPCRAFT_SETUP_PROXIES', 'http_proxy')])

    def build(self):
        super().build()

        mvn_cmd = ['mvn', 'package']
        if self._use_proxy():
            settings_path = os.path.join(self.partdir, 'm2', 'settings.xml')
            _create_settings(settings_path)
            mvn_cmd += ['-s', settings_path]

        self.run(mvn_cmd + self.options.maven_options)

        for f in self.options.maven_targets:
            src = os.path.join(self.builddir, f, 'target')

            jarfiles = glob.glob(os.path.join(src, '*.jar'))
            warfiles = glob.glob(os.path.join(src, '*.war'))
            arfiles = glob.glob(os.path.join(src, '*.[jw]ar'))

            if not (arfiles):
                raise RuntimeError("could not find any"
                                   "built jar files for part")
            basedir = 'jar' if jarfiles and len(f) == 0 else (
                      'war' if warfiles and len(f) == 0 else f)

            targetdir = os.path.join(self.installdir, basedir)
            os.makedirs(targetdir, exist_ok=True)
            for f in arfiles:
                shutil.copy(f, targetdir)



def _create_settings(settings_path):
    proxy = urlparse(os.environ['http_proxy'])
    os.makedirs(os.path.dirname(settings_path), exist_ok=True)
    with open(settings_path, 'w') as f:
        f.write(_MVN_SETTINGS_FORMAT.format(
            proxy.hostname,
            proxy.port,
            _get_no_proxy_string()))


def _get_no_proxy_string():
    no_proxy = [k.strip() for k in
                os.environ.get('no_proxy', 'localhost').split(',')]
    return '|'.join(no_proxy)
