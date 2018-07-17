# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import os

import snapcraft
from snapcraft.internal import mangling


class PlainboxProviderPlugin(snapcraft.BasePlugin):
    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(["intltool"])
        self.stage_packages.extend(
            ["python3-pip", "python3-wheel", "python3-setuptools"]
        )

    def build(self):
        super().build()

        env = os.environ.copy()
        # Ensure the first provider does not attempt to validate against
        # providers installed on the build host by initialising PROVIDERPATH
        # to empty
        env["PROVIDERPATH"] = ""
        provider_stage_dir = os.path.join(self.project.stage_dir, "providers")
        if os.path.exists(provider_stage_dir):
            provider_dirs = [
                os.path.join(provider_stage_dir, provider)
                for provider in os.listdir(provider_stage_dir)
            ]
            env["PROVIDERPATH"] = ":".join(provider_dirs)
        self.run(["python3", "manage.py", "validate"], env=env)
        self.run(["python3", "manage.py", "build"])
        self.run(["python3", "manage.py", "i18n"])
        self.run(
            [
                "python3",
                "manage.py",
                "install",
                "--layout=relocatable",
                "--prefix=/providers/{}".format(self.name),
                "--root={}".format(self.installdir),
            ]
        )

        mangling.rewrite_python_shebangs(self.installdir)

    def snap_fileset(self):
        fileset = super().snap_fileset()
        # If a python package is added as a stage-packages it will include
        # sitecustomize.py which is irrelevant and will cause unnecessary
        # conflicts so instead we just ignore these entries.
        fileset.append("-usr/lib/python*/sitecustomize.py")
        fileset.append("-etc/python*/sitecustomize.py")
        return fileset
