# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import os.path
import subprocess
from textwrap import dedent
from unittest import mock

from testtools.matchers import Contains, Equals, FileExists
from . import CommandBaseTestCase


class PackCommandBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        patcher = mock.patch(
            "snapcraft.internal.lifecycle._packer.Popen",
            new=mock.Mock(wraps=subprocess.Popen),
        )
        self.popen_spy = patcher.start()
        self.addCleanup(patcher.stop)


class PackCommandTestCase(PackCommandBaseTestCase):

    scenarios = (
        ("pack", dict(command="pack")),
        ("deprecated snap", dict(command="snap")),
    )

    def setUp(self):
        super().setUp()
        self.snap_dir = "mysnap"
        self.meta_dir = os.path.join(self.snap_dir, "meta")
        os.makedirs(self.meta_dir)
        self.snap_yaml = os.path.join(self.meta_dir, "snap.yaml")

    def test_snap_from_dir(self):
        with open(self.snap_yaml, "w") as f:
            f.write(
                dedent(
                    """\
                name: my_snap
                version: 99
                architectures: [amd64, armhf]
            """
                )
            )

        result = self.run_command([self.command, self.snap_dir])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Snapped my_snap_99_multi.snap\n"))

        self.popen_spy.assert_called_once_with(
            [
                "mksquashfs",
                "mysnap",
                "my_snap_99_multi.snap",
                "-noappend",
                "-comp",
                "xz",
                "-no-xattrs",
                "-no-fragments",
                "-all-root",
            ],
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )

        self.assertThat("my_snap_99_multi.snap", FileExists())

    def test_snap_from_dir_with_no_arch(self):
        with open(self.snap_yaml, "w") as f:
            f.write(
                dedent(
                    """\
                name: my_snap
                version: 99
            """
                )
            )

        result = self.run_command([self.command, self.snap_dir])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Snapped my_snap_99_all.snap\n"))

        self.popen_spy.assert_called_once_with(
            [
                "mksquashfs",
                "mysnap",
                "my_snap_99_all.snap",
                "-noappend",
                "-comp",
                "xz",
                "-no-xattrs",
                "-no-fragments",
                "-all-root",
            ],
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )

        self.assertThat("my_snap_99_all.snap", FileExists())

    def test_snap_from_dir_type_os_does_not_use_all_root(self):
        with open(self.snap_yaml, "w") as f:
            f.write(
                dedent(
                    """\
                name: my_snap
                version: 99
                architectures: [amd64, armhf]
                type: os
            """
                )
            )

        result = self.run_command([self.command, self.snap_dir])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Snapped my_snap_99_multi.snap\n"))

        self.popen_spy.assert_called_once_with(
            [
                "mksquashfs",
                "mysnap",
                "my_snap_99_multi.snap",
                "-noappend",
                "-comp",
                "xz",
                "-no-xattrs",
                "-no-fragments",
            ],
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )

        self.assertThat("my_snap_99_multi.snap", FileExists())
