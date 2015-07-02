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

import os
import snapcraft.common
import subprocess
import sys


class BasePlugin:

    def __init__(self, name, options):
        self.name = name
        self.options = options
        self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
    def pull(self):
        return True

    def build(self):
        return True

    def stage(self):
        return True

    def snap(self):
        return True

    def test(self):
        return True

    def env(self):
        return []

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if cwd is None:
            cwd = self.builddir
        if True:
            if isinstance(cmd, list):
                print(' '.join(cmd))
            else:
                print(cmd)
        return snapcraft.common.run(cmd, cwd=cwd, **kwargs)

    def pullBranch(self, url):
        if url.startswith("bzr:") or url.startswith("lp:"):
            if os.path.exists(os.path.join(self.sourcedir, ".bzr")):
                return self.run("bzr pull " + url, self.sourcedir)
            else:
                os.rmdir(self.sourcedir)
                return self.run("bzr branch " + url + " " + self.sourcedir)
        elif url.startswith("git:"):
            if os.path.exists(os.path.join(self.sourcedir, ".git")):
                return self.run("git pull", self.sourcedir)
            else:
                return self.run("git clone " + url + " .", self.sourcedir)
        else:
            raise Exception("Did not recognize branch url: " + url)

    def doDeploy(self, dirs):
        self.makedirs(self.snapdir)

        for d in dirs:
            if os.path.exists(os.path.join(self.stagedir, d)):
                self.makedirs(os.path.join(self.snapdir, d))
                if not self.run("cp -rf " + d + " " + self.snapdir + "/", cwd=self.stagedir):
                    return False
        return True

    def makedirs(self, d):
        try:
            os.makedirs(d)
        except FileExistsError:
            pass
