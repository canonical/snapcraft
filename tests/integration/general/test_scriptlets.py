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

import multiprocessing
import os

from testtools.matchers import FileContains, FileExists, Not

from tests import integration


class ScriptletTestCase(integration.TestCase):
    def test_prepare_scriptlet(self):
        self.run_snapcraft("build", "scriptlet-prepare")

        installdir = os.path.join(self.parts_dir, "prepare-scriptlet-test", "install")
        touch_file_path = os.path.join(installdir, "prepared")
        self.assertThat(touch_file_path, FileExists())

    def test_build_scriptlet(self):
        self.run_snapcraft("build", "scriptlet-build")

        partdir = os.path.join(self.parts_dir, "build-scriptlet-test")
        builddir = os.path.join(partdir, "build")
        installdir = os.path.join(partdir, "install")

        touch_file_path = os.path.join(builddir, "build-build")
        self.assertThat(touch_file_path, FileExists())
        jobs_file_name = "jobs-{}".format(multiprocessing.cpu_count())
        touch_file_path = os.path.join(builddir, jobs_file_name)
        self.assertThat(touch_file_path, FileExists())
        touch_file_path = os.path.join(installdir, "build-install")
        self.assertThat(touch_file_path, FileExists())

    def test_install_scriptlet(self):
        self.run_snapcraft("build", "scriptlet-install")

        installdir = os.path.join(self.parts_dir, "install-scriptlet-test", "install")
        touch_file_path = os.path.join(installdir, "build-done")
        self.assertThat(touch_file_path, FileExists())
        echoed_file_path = os.path.join(installdir, "config.ini")
        self.assertThat(echoed_file_path, FileExists())
        self.assertThat(echoed_file_path, FileContains("config-key=config-value\n"))
        arch_triplet_file = os.path.join(installdir, "lib", self.arch_triplet, "lib.so")
        self.assertThat(arch_triplet_file, FileExists())

    def test_override_pull(self):
        self.run_snapcraft("pull", "scriptlet-override-pull")

        sourcedir = os.path.join(self.parts_dir, "override-pull-scriptlet-test", "src")

        self.assertThat(os.path.join(sourcedir, "file"), FileExists())
        self.assertThat(os.path.join(sourcedir, "before-pull"), FileExists())
        self.assertThat(os.path.join(sourcedir, "after-pull"), FileExists())

    def test_override_build(self):
        self.run_snapcraft("build", "scriptlet-override-build")

        builddir = os.path.join(
            self.parts_dir, "override-build-scriptlet-test", "build"
        )
        installdir = os.path.join(
            self.parts_dir, "override-build-scriptlet-test", "install"
        )

        self.assertThat(os.path.join(builddir, "file"), FileExists())
        self.assertThat(os.path.join(builddir, "before-build"), FileExists())
        self.assertThat(os.path.join(builddir, "after-build"), FileExists())
        self.assertThat(os.path.join(installdir, "file"), FileExists())
        self.assertThat(os.path.join(installdir, "before-install"), FileExists())
        self.assertThat(os.path.join(installdir, "after-install"), FileExists())

    def test_override_stage(self):
        self.run_snapcraft("stage", "scriptlet-override-stage")

        installdir = os.path.join(
            self.parts_dir, "override-stage-scriptlet-test", "install"
        )

        # Assert that the before/after stage files aren't placed in the
        # installdir, although the file is.
        self.assertThat(os.path.join(installdir, "file"), FileExists())
        self.assertThat(os.path.join(installdir, "before-stage"), Not(FileExists()))
        self.assertThat(os.path.join(installdir, "after-stage"), Not(FileExists()))

        self.assertThat(os.path.join(self.stage_dir, "file"), FileExists())
        self.assertThat(os.path.join(self.stage_dir, "before-stage"), FileExists())
        self.assertThat(os.path.join(self.stage_dir, "after-stage"), FileExists())

        # Also assert that, while file2 was installed, it wasn't staged
        self.assertThat(
            os.path.join(
                self.parts_dir, "override-stage-do-nothing", "install", "file2"
            ),
            FileExists(),
        )
        self.assertThat(os.path.join(self.stage_dir, "file2"), Not(FileExists()))

    def test_override_prime(self):
        self.run_snapcraft("prime", "scriptlet-override-prime")

        installdir = os.path.join(
            self.parts_dir, "override-prime-scriptlet-test", "install"
        )

        # Assert that the before/after prime files aren't placed in the
        # installdir, although the file is.
        self.assertThat(os.path.join(installdir, "file"), FileExists())
        self.assertThat(os.path.join(installdir, "before-prime"), Not(FileExists()))
        self.assertThat(os.path.join(installdir, "after-prime"), Not(FileExists()))

        # Assert that the before/after prime files aren't placed in the
        # stagedir, although the file is.
        self.assertThat(os.path.join(self.stage_dir, "file"), FileExists())
        self.assertThat(os.path.join(self.stage_dir, "before-prime"), Not(FileExists()))
        self.assertThat(os.path.join(self.stage_dir, "after-prime"), Not(FileExists()))

        self.assertThat(os.path.join(self.prime_dir, "file"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "before-prime"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "after-prime"), FileExists())

        # Also assert that, while file2 was installed and staged, it wasn't
        # primed
        self.assertThat(
            os.path.join(
                self.parts_dir, "override-prime-do-nothing", "install", "file2"
            ),
            FileExists(),
        )
        self.assertThat(os.path.join(self.stage_dir, "file2"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "file2"), Not(FileExists()))
