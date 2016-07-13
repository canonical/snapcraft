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
"""

import snapcraft


class PlainboxProviderPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(['intltool'])

    def build(self):
        super().build()
        self.run(["python3", "manage.py", "build"])
        self.run(["python3", "manage.py", "i18n"])
        self.run([
            "python3", "manage.py", "install", "--layout=relocatable",
            "--prefix=/providers/{}".format(self.name),
            "--root={}".format(self.installdir)])
