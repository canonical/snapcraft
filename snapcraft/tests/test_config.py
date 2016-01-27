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
from __future__ import absolute_import, unicode_literals
import os
import shutil
import tempfile
from configparser import ConfigParser

from mock import patch

from snapcraft import tests
from snapcraft.config import clear_config, load_config, save_config


class ConfigTestCase(tests.TestCase):

    def setUp(self):
        super(ConfigTestCase, self).setUp()

        patcher = patch('snapcraft.config.load_first_config')
        self.mock_load_first_config = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('snapcraft.config.save_config_path')
        self.mock_save_config_path = patcher.start()
        self.addCleanup(patcher.stop)

        cfg_dir = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(cfg_dir))
        self.filename = os.path.join(cfg_dir, 'snapcraft.cfg')
        self.mock_load_first_config.return_value = self.filename
        self.mock_save_config_path.return_value = cfg_dir

        # make sure env is not overwritten
        patcher = patch.object(os, 'environ', {})
        patcher.start()
        self.addCleanup(patcher.stop)

    def get_temporary_file(self, suffix='.cfg'):
        return tempfile.NamedTemporaryFile(suffix=suffix)


class LoadConfigTestCase(ConfigTestCase):

    def test_load_config_with_no_existing_file(self):
        data = load_config()
        self.assertEqual(data, {})

    def test_load_config_with_no_existing_section(self):
        cfg = ConfigParser()
        cfg.add_section('some.domain')
        cfg.set('some.domain', 'foo', '1')
        with open(self.filename, 'w') as fd:
            cfg.write(fd)

        data = load_config()
        self.assertEqual(data, {})

    def test_load_config(self):
        cfg = ConfigParser()
        cfg.add_section('login.ubuntu.com')
        cfg.set('login.ubuntu.com', 'foo', '1')
        with open(self.filename, 'w') as fd:
            cfg.write(fd)

        data = load_config()
        self.assertEqual(data, {'foo': '1'})


class SaveConfigTestCase(ConfigTestCase):

    def test_save_config_with_no_existing_file(self):
        data = {'key': 'value'}

        save_config(data)
        self.assertEqual(load_config(), data)

    def test_save_config_with_existing_file(self):
        cfg = ConfigParser()
        cfg.add_section('some.domain')
        cfg.set('some.domain', 'foo', '1')
        with open(self.filename, 'w') as fd:
            cfg.write(fd)

        data = {'key': 'value'}
        save_config(data)

        config = load_config()
        self.assertEqual(config, data)


class ClearConfigTestCase(ConfigTestCase):

    def test_clear_config_with_no_existing_section(self):
        cfg = ConfigParser()
        cfg.add_section('some.domain')
        cfg.set('some.domain', 'foo', '1')
        with open(self.filename, 'w') as fd:
            cfg.write(fd)

        config = load_config()
        assert config == {}

        clear_config()

        config = load_config()
        self.assertEqual(config, {})

    def test_clear_config_removes_existing_section(self):
        cfg = ConfigParser()
        cfg.add_section('login.ubuntu.com')
        cfg.set('login.ubuntu.com', 'foo', '1')
        with open(self.filename, 'w') as fd:
            cfg.write(fd)

        config = load_config()
        assert config != {}

        clear_config()

        config = load_config()
        self.assertEqual(config, {})

    def test_clear_config_with_no_existing_file(self):
        config = load_config()
        assert config == {}

        clear_config()

        config = load_config()
        self.assertEqual(config, {})
