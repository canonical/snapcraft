# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import fixtures
import os

from testtools.matchers import Equals

from snapcraft import (
    config,
    tests
)


def create_config_from_string(content):
    path = config.Config.save_path()
    with open(path, 'w') as f:
        f.write(content)


class TestConfig(tests.TestCase):

    def test_non_existing_file_succeeds(self):
        conf = config.Config()
        self.assertThat(conf.parser.sections(), Equals([]))
        self.assertTrue(conf.is_empty())

    def test_existing_file(self):
        existing_conf = config.Config()
        existing_conf.set('foo', 'bar')
        existing_conf.save()
        # Check we find and use the existing conf
        conf = config.Config()
        self.assertThat(conf.get('foo'), Equals('bar'))
        self.assertFalse(conf.is_empty())

    def test_irrelevant_sections_are_ignored(self):
        create_config_from_string('''[example.com]\nfoo=bar''')
        conf = config.Config()
        self.assertThat(conf.get('foo'), Equals(None))

    def test_section_from_url(self):
        create_config_from_string('''[example.com]\nfoo=bar''')
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL', 'http://example.com/api/v2'))
        conf = config.Config()
        self.assertThat(conf.get('foo'), Equals('bar'))

    def test_save_one_option(self):
        conf = config.Config()
        conf.set('bar', 'baz')
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get('bar'), Equals('baz'))

    def test_clear_preserver_other_sections(self):
        create_config_from_string('''[keep_me]\nfoo=bar\n''')
        conf = config.Config()
        conf.set('bar', 'baz')
        self.assertThat(conf.get('bar'), Equals('baz'))
        conf.clear()
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get('bar'), Equals(None))
        # Picking behind the curtains
        self.assertThat(new_conf.parser.get('keep_me', 'foo'), Equals('bar'))
        self.assertTrue(conf.is_empty())


class TestOptions(tests.TestCase):

    def create_config(self, **kwargs):
        conf = config.Config()
        for k, v in kwargs.items():
            conf.set(k, v)
        return conf

    def test_string(self):
        conf = self.create_config(foo='bar')
        self.assertThat(conf.get('foo'), Equals('bar'))


def create_local_config_from_string(content):
    os.makedirs(os.path.dirname(config.LOCAL_CONFIG_FILENAME), exist_ok=True)
    with open(config.LOCAL_CONFIG_FILENAME, 'w') as f:
        f.write(content)


class TestLocalConfig(tests.TestCase):

    def setUp(self):
        super().setUp()
        override_sso = fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL', 'http://example.com/api/v2')
        self.useFixture(override_sso)

    def test_local_config_is_considered(self):
        create_local_config_from_string('''[example.com]\nfoo=bar''')
        conf = config.Config()
        self.assertThat(conf.get('foo'), Equals('bar'))

    def test_local_config_is_preferred(self):
        create_config_from_string('''[example.com]\nfoo=baz''')
        create_local_config_from_string('''[example.com]\nfoo=bar''')
        conf = config.Config()
        self.assertThat(conf.get('foo'), Equals('bar'))

    def test_local_config_is_static(self):
        create_local_config_from_string('''[example.com]\nfoo=bar''')
        conf = config.Config()
        conf.set('foo', 'baz')
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get('foo'), Equals('bar'))
