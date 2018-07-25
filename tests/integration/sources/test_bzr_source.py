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

import subprocess

from testtools.matchers import Equals

from tests import integration


class BzrSourceTestCase(integration.BzrSourceBaseTestCase):
    def test_pull_bzr_head(self):
        self.copy_project_to_cwd("bzr-head")

        self.init_source_control()
        self.commit('"1"', unchanged=True)
        self.commit('"2"', unchanged=True)
        # test initial branch
        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            ["bzr", "revno", "-r", "-1", "parts/bzr/src"], universal_newlines=True
        ).strip()
        self.assertThat(revno, Equals("2"))
        # test pull doesn't fail
        self.run_snapcraft("pull")
        revno = subprocess.check_output(
            ["bzr", "revno", "-r", "-1", "parts/bzr/src"], universal_newlines=True
        ).strip()
        self.assertThat(revno, Equals("2"))

    def test_pull_bzr_tag(self):
        self.copy_project_to_cwd("bzr-tag")

        self.init_source_control()
        self.commit('"1"', unchanged=True)
        self.commit('"2"', unchanged=True)
        subprocess.check_call(
            ["bzr", "tag", "-r", "1", "initial"], stderr=subprocess.DEVNULL
        )
        # test initial branch
        self.run_snapcraft("pull")
        revno = self.get_revno("parts/bzr/src")
        self.assertThat(revno, Equals("1"))
        # test pull doesn't fail
        self.run_snapcraft("pull")
        revno = self.get_revno("parts/bzr/src")
        self.assertThat(revno, Equals("1"))

    def test_pull_bzr_commit(self):
        self.copy_project_to_cwd("bzr-commit")

        self.init_source_control()
        self.commit('"1"', unchanged=True)
        self.commit('"2"', unchanged=True)
        # test initial branch
        self.run_snapcraft("pull")
        revno = self.get_revno("parts/bzr/src")
        self.assertThat(revno, Equals("1"))
        # test pull doesn't fail
        self.run_snapcraft("pull")
        revno = self.get_revno("parts/bzr/src")
        self.assertThat(revno, Equals("1"))
