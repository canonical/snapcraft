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

""" Create parts containing a Plainbox test collection known as a provider.

Plainbox is a toolkit consisting of python3 library, development tools,
documentation and examples. It is targeted at developers working on testing or
certification applications and authors creating tests for such applications.
More information: http://plainbox.readthedocs.org/en/latest/

To find out more about authoring a plainbox provider, see the following
documentation: http://plainbox.readthedocs.org/en/latest/author/providers.html

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Plugin specific keywords:
    - checkbox-dev-ppa:
      (boolean)
      Whether to extend the source list to include the Checkbox Development
      PPA https://launchpad.net/~checkbox-dev/+archive/ubuntu/ppa. Defaults to
      true as features to support Snappy Ubuntu Core have yet to land in the
      distro repositories.
"""

import snapcraft


class PlainboxProviderPlugin(snapcraft.BasePlugin):

    _DEV_PPA_PLUGIN_STAGE_SOURCES = '''
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} multiverse
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates multiverse
deb http://${security}.ubuntu.com/${suffix} ${release}-security main restricted
deb http://${security}.ubuntu.com/${suffix} ${release}-security universe
deb http://${security}.ubuntu.com/${suffix} ${release}-security multiverse
deb http://ppa.launchpad.net/checkbox-dev/ppa/ubuntu ${release} main
'''

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['checkbox-dev-ppa'] = {
            'type': 'boolean',
            'default': 'true',
        }
        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(['python3-plainbox', 'intltool'])
        if self.options.checkbox_dev_ppa:
            self._PLUGIN_STAGE_SOURCES = self._DEV_PPA_PLUGIN_STAGE_SOURCES

    def build(self):
        super().build()
        self.run(["python3", "manage.py", "build"])
        self.run(["python3", "manage.py", "i18n"])
        self.run([
            "python3", "manage.py", "install", "--layout=relocatable",
            "--prefix=/providers/{}".format(self.name),
            "--root={}".format(self.installdir)])
