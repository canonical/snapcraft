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

import subprocess

from snapcraft.tests import integration, fixture_setup
from testtools.matchers import Contains


class PartsGrammarTestCase(integration.TestCase):

    def setUp(self):
        super().setUp()

    def test_plain_source_string(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part('my-part', {
            'plugin': 'nil',
            'source': 'https://github.com/snapcrafters/fork-and-rename-me.git',
        })
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft(['pull'])

    def test_source_on_current_arch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part('my-part', {
            'plugin': 'nil',
            'source': [
                {'on {}'.format(self.deb_arch): '.'},
                {'else': 'invalid'},
            ]
        })
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft(['pull'])

    def test_source_on_other_arch_else(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part('my-part', {
            'plugin': 'nil',
            'source': [
                {'on other-arch': 'invalid'},
                {'else': '.'},
            ]
        })
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft(['pull'])

    def test_source_on_other_arch_else_fail(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part('my-part', {
            'plugin': 'nil',
            'source': [
                {'on other-arch': 'invalid'},
                'else fail',
            ]
        })
        self.useFixture(snapcraft_yaml)
        self.assertThat(self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['pull']).output, Contains(
                "Unable to satisfy 'on other-arch', failure forced"))
