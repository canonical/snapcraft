# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import subprocess

from testtools.matchers import Equals, FileExists

from tests import integration


class SubversionSourceTestCase(integration.SubversionSourceBaseTestCase):
    def test_pull_svn_checkout(self):
        self.copy_project_to_cwd("svn-pull")

        self.init_source_control()
        self.checkout("file:///{}".format(os.path.join(self.path, "repo")), "local")
        open(os.path.join("local", "file"), "w").close()
        self.add("file", cwd="local/")
        self.commit("test", cwd="local/")
        self.update(cwd="local/")
        subprocess.check_call(["rm", "-rf", "local/"], stdout=subprocess.DEVNULL)

        self.run_snapcraft("pull")
        part_src_path = os.path.join(self.parts_dir, "svn", "src")
        revno = subprocess.check_output(["svnversion", part_src_path]).strip()
        self.assertThat(revno, Equals(b"1"))
        self.assertThat(os.path.join(part_src_path, "file"), FileExists())

    def test_pull_svn_update(self):
        self.copy_project_to_cwd("svn-pull-update")

        self.init_source_control()

        self.checkout("file:///{}".format(os.path.join(self.path, "repo")), "local")
        open(os.path.join("local", "file"), "w").close()
        self.add("file", cwd="local/")
        self.commit("test", cwd="local/")
        self.update(cwd="local/")
        subprocess.check_call(["rm", "-rf", "local/"], stdout=subprocess.DEVNULL)

        part_src_path = os.path.join(self.parts_dir, "svn", "src")
        self.checkout(
            "file:///{}".format(os.path.join(self.path, "repo")), part_src_path
        )
        self.checkout("file:///{}".format(os.path.join(self.path, "repo")), "local")
        open(os.path.join("local", "filetwo"), "w").close()
        self.add("filetwo", cwd="local/")
        self.commit("testtwo", cwd="local/")
        self.update(cwd="local/")
        subprocess.check_call(["rm", "-rf", "local/"], stdout=subprocess.DEVNULL)

        self.run_snapcraft("pull")
        revno = subprocess.check_output(["svnversion", part_src_path]).strip()
        self.assertThat(revno, Equals(b"2"))
        self.assertThat(os.path.join(part_src_path, "file"), FileExists())
        self.assertThat(os.path.join(part_src_path, "filetwo"), FileExists())
