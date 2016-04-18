# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from xdg import BaseDirectory

import fixtures

from snapcraft import (
    config,
    tests,
)


class TestConfig(tests.TestCase):

    def setUp(self):
        super().setUp()
        # Tell xdg to look into our private tmp dir
        self.addCleanup(
            setattr, BaseDirectory, 'xdg_config_home',
            BaseDirectory.xdg_config_home)
        self.addCleanup(
            setattr, BaseDirectory, 'xdg_config_dirs',
            BaseDirectory.xdg_config_dirs)
        BaseDirectory.xdg_config_home = os.path.join(self.path, '.config')
        BaseDirectory.xdg_config_dirs = [BaseDirectory.xdg_config_home]

    def create_config_from_string(self, content):
        path = config.Config.save_path()
        with open(path, 'w') as f:
            f.write(content)

    def test_non_existing_file_succeeds(self):
        conf = config.Config()
        conf.load()
        self.assertEqual([], conf.parser.sections())

    def test_existing_file(self):
        existing_conf = config.Config()
        existing_conf.set('foo', 'bar')
        existing_conf.save()
        # Check we find and use the existing conf
        conf = config.Config()
        conf.load()
        self.assertEqual('bar', conf.get('foo'))

    def test_irrelevant_sections_are_ignored(self):
        self.create_config_from_string('''[example.com]\nfoo=bar''')
        conf = config.Config()
        conf.load()
        self.assertEqual(None, conf.get('foo'))

    def test_section_from_url(self):
        self.create_config_from_string('''[example.com]\nfoo=bar''')
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL', 'http://example.com/api/v2'))
        conf = config.Config()
        conf.load()
        self.assertEqual('bar', conf.get('foo'))

    def test_save_one_option(self):
        conf = config.Config()
        conf.set('bar', 'baz')
        conf.save()
        new_conf = config.Config()
        new_conf.load()
        self.assertEqual('baz', conf.get('bar'))

    def test_clear_preserver_other_sections(self):
        self.create_config_from_string('''[keep_me]\nfoo=bar\n''')
        conf = config.Config()
        conf.load()
        conf.set('bar', 'baz')
        self.assertEqual('baz', conf.get('bar'))
        conf.clear()
        conf.save()
        new_conf = config.Config()
        new_conf.load()
        self.assertEqual(None, conf.get('bar'))
        # Picking behind the curtains
        self.assertEqual('bar', conf.parser.get('keep_me', 'foo'))
