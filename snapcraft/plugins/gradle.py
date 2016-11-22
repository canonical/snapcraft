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

"""This plugin is useful for building parts that use gradle.

The gradle build system is commonly used to build Java projects.
The plugin requires a pom.xml in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - gradle-options:
      (list of strings)
      flags to pass to the build using the gradle semantics for parameters.
"""

import glob
import logging
import os
import urllib.parse
import snapcraft
import snapcraft.common
import snapcraft.plugins.jdk


logger = logging.getLogger(__name__)


class GradlePlugin(snapcraft.plugins.jdk.JdkPlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['gradle-options'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['build-properties'].append('gradle-options')

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('ca-certificates-java')

    def build(self):
        super().build()

        gradle_cmd = ['./gradlew']
        self.run(gradle_cmd +
                 self._get_proxy_options() +
                 self.options.gradle_options + ['jar'])

        src = os.path.join(self.builddir, 'build', 'libs')
        jarfiles = glob.glob(os.path.join(src, '*.jar'))
        warfiles = glob.glob(os.path.join(src, '*.war'))

        if len(jarfiles) > 0:
            basedir = 'jar'
        elif len(warfiles) > 0:
            basedir = 'war'
            jarfiles = warfiles
        else:
            raise RuntimeError("Could not find any "
                               "built jar files for part")

        targetdir = os.path.join(self.installdir, basedir)
        os.makedirs(targetdir, exist_ok=True)
        for f in jarfiles:
            base = os.path.basename(f)
            os.link(f, os.path.join(targetdir, base))

    def _get_proxy_options(self):
        # XXX This doesn't yet support username and password.
        # -- elopio - 2016-11-17
        proxy_options = []
        for var in ('http', 'https'):
            proxy = os.environ.get('{}_proxy'.format(var), False)
            if proxy:
                parsed_url = urllib.parse.urlparse(proxy)
                proxy_options.append('-D{}.proxyHost={}'.format(
                    var, parsed_url.hostname))
                if parsed_url.port:
                    proxy_options.append(
                        '-D{}.proxyPort={}'.format(var, parsed_url.port))
        return proxy_options
