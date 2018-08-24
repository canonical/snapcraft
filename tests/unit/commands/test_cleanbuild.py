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

from textwrap import dedent
from unittest.mock import patch, ANY

from testtools.matchers import Equals, FileExists

from . import CommandBaseTestCase


class CleanBuildCommandBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            dedent(
                """\
            name: snap-test
            version: 1.0
            summary: test cleanbuild
            description: if snap is successful a snap package will be available
            architectures: ['amd64']
            confinement: strict
            grade: stable

            parts:
                part1:
                  plugin: nil
        """
            )
        )

        patcher = patch("snapcraft.internal.lxd.Cleanbuilder")
        self.cleanbuilder_mock = patcher.start()
        self.addCleanup(patcher.stop)


class CleanBuildCommandTestCase(CleanBuildCommandBaseTestCase):
    def test_cleanbuild(self):
        result = self.run_command(["cleanbuild"])

        self.assertThat(result.exit_code, Equals(0))
        self.cleanbuilder_mock.assert_called_once_with(
            project=ANY, remote=None, source="snap-test_source.tar.bz2"
        )
        self.assertThat("snap-test_source.tar.bz2", FileExists())

    def test_cleanbuild_debug_appended_works(self):
        result = self.run_command(["cleanbuild", "--debug"])

        self.assertThat(result.exit_code, Equals(0))
        self.cleanbuilder_mock.assert_called_once_with(
            project=ANY, remote=None, source="snap-test_source.tar.bz2"
        )

    def test_cleanbuild_debug_prepended_works(self):
        result = self.run_command(["--debug", "cleanbuild"])

        self.assertThat(result.exit_code, Equals(0))
        self.cleanbuilder_mock.assert_called_once_with(
            project=ANY, remote=None, source="snap-test_source.tar.bz2"
        )
