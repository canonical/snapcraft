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

import fixtures
from xdg import BaseDirectory
from textwrap import dedent
from unittest import mock
from unittest.mock import call

from testtools.matchers import Equals
from snapcraft.tests import TestWithFakeRemoteParts
from snapcraft.tests import fixture_setup
from . import CommandBaseTestCase
from snapcraft.internal.errors import SnapcraftEnvironmentError


class RefreshCommandBaseTestCase(CommandBaseTestCase, TestWithFakeRemoteParts):

    yaml_template = dedent("""\
        name: snap-test
        version: 1.0
        summary: test snapping
        description: if snap is succesful a snap package will be available
        architectures: ['amd64']
        type: {}
        confinement: strict
        grade: stable

        parts:
            part1:
                plugin: nil
        """)

    def setUp(self):
        super().setUp()
        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')
        self.headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')

    def make_snapcraft_yaml(self, n=1, snap_type='app', snapcraft_yaml=None):
        if not snapcraft_yaml:
            snapcraft_yaml = self.yaml_template.format(snap_type)
        super().make_snapcraft_yaml(snapcraft_yaml)
        self.state_dir = os.path.join(self.parts_dir, 'part1', 'state')


class RefreshCommandTestCase(RefreshCommandBaseTestCase):

    scenarios = [
         ('local', dict(snapcraft_container_builds='1', remote='local')),
         ('remote', dict(snapcraft_container_builds='foo', remote='foo')),
    ]

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_refresh(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_filesystem = fixture_setup.FakeFilesystem()
        self.useFixture(fake_filesystem)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_CONTAINER_BUILDS', self.snapcraft_container_builds))
        self.make_snapcraft_yaml()

        self.run_command(['refresh'])

        mock_container_run.assert_has_calls([
            call(['apt-get', 'update']),
            call(['apt-get', 'upgrade', '-y']),
            call(['snap', 'refresh']),
        ])
        self.assertThat(fake_lxd.name,
                        Equals('{}:snapcraft-snap-test'.format(self.remote)))


class RefreshCommandErrorsTestCase(RefreshCommandBaseTestCase):

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_refresh_fails_without_env_var(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_filesystem = fixture_setup.FakeFilesystem()
        self.useFixture(fake_filesystem)
        self.make_snapcraft_yaml()

        self.assertRaises(SnapcraftEnvironmentError,
                          self.run_command,
                          ['refresh'])
        mock_container_run.assert_not_called()
