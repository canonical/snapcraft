# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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


class HgSourceTestCase(integration.HgSourceBaseTestCase):
    def test_pull_hg_head(self):
        self.copy_project_to_cwd("hg-head")

        self.init_source_control()
        open("1", "w").close()
        self.commit("1", "1")
        open("2", "w").close()
        self.commit("2", "2")

        self.run_snapcraft("pull")
        revno = self.get_revno(os.path.join(self.parts_dir, "mercurial", "src"))
        self.assertThat(revno, Equals('"2"'))

        self.run_snapcraft("pull")
        revno = self.get_revno(os.path.join(self.parts_dir, "mercurial", "src"))
        self.assertThat(revno, Equals('"2"'))

    def test_pull_hg_tag(self):
        self.copy_project_to_cwd("hg-tag")

        self.init_source_control()
        open("1", "w").close()
        self.commit("1", "1")
        subprocess.check_call(["hg", "tag", "initial", "--user", '"Example Dev"'])
        open("2", "w").close()
        self.commit("2", "2")

        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            "ls -1 {} | wc -l ".format(
                os.path.join(self.parts_dir, "mercurial", "src")
            ),
            shell=True,
            universal_newlines=True,
        ).strip()
        self.assertThat(revno, Equals("1"))

        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            "ls -1 {} | wc -l ".format(
                os.path.join(self.parts_dir, "mercurial", "src")
            ),
            shell=True,
            universal_newlines=True,
        ).strip()
        self.assertThat(revno, Equals("1"))

    def test_pull_hg_commit(self):
        self.copy_project_to_cwd("hg-commit")

        self.init_source_control()
        open("1", "w").close()
        self.commit("1", "1")
        open("2", "w").close()
        self.commit("2", "2")

        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            "ls -1 {} | wc -l ".format(
                os.path.join(self.parts_dir, "mercurial", "src")
            ),
            shell=True,
            universal_newlines=True,
        ).strip()
        self.assertThat(revno, Equals("1"))

        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            "ls -1 {} | wc -l ".format(
                os.path.join(self.parts_dir, "mercurial", "src")
            ),
            shell=True,
            universal_newlines=True,
        ).strip()
        self.assertThat(revno, Equals("1"))

    def test_pull_hg_branch(self):
        self.copy_project_to_cwd("hg-branch")

        self.init_source_control()
        subprocess.check_call(["hg", "branch", "second"], stdout=subprocess.DEVNULL)
        open("second", "w").close()
        self.commit("second", "second")
        subprocess.check_call(["hg", "branch", "default"], stdout=subprocess.DEVNULL)
        open("default", "w").close()
        self.commit("default", "default")

        self.run_snapcraft("pull")
        self.assertThat(
            os.path.join(self.parts_dir, "mercurial", "src", "second"), FileExists()
        )

        self.run_snapcraft("pull")
        self.assertThat(
            os.path.join(self.parts_dir, "mercurial", "src", "second"), FileExists()
        )
