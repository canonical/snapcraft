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

import testscenarios

import integration_tests
import snapcraft
from snapcraft.tests import fixture_setup


class BuildSnapsTestCase(
        testscenarios.WithScenarios, integration_tests.TestCase):

    scenarios = (
        ('snap name', {'snap': 'u1test-snap-with-tracks'}),
        ('snap name with track and risk',
         {'snap': 'u1test-snap-with-tracks/test-track-1/beta'}))

    def test_build_snap(self):
        self.useFixture(fixture_setup.WithoutSnapInstalled(self.snap))
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            'test-part-with-build-snap', {
                'plugin': 'nil',
                'build-snaps': [self.snap]
            })
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft('build')
        self.assertTrue(
            snapcraft.repo.snaps.SnapPackage.is_snap_installed(self.snap))
