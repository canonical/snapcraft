# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import click
from testtools.matchers import Equals

from snapcraft.project.errors import YamlValidationError
import snapcraft.cli._config as config
from tests import unit


class TestConfig(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.config_path = os.path.join(click.get_app_dir("snapcraft"), "config.yaml")

    def test_config_path(self):
        path = config._get_global_config_path()
        self.assertThat(path, Equals(self.config_path))

    def test_convert_key_dashes_to_underscores(self):
        data = {"a-b-c": True, "x_y_z": False}
        data = config._convert_key_dashes_to_underscores(data)
        expected = {"a_b_c": True, "x_y_z": False}
        self.assertThat(data, Equals(expected))

    def test_load_config(self):
        dir_path = os.path.dirname(self.config_path)
        os.makedirs(dir_path)
        with open(self.config_path, "w") as f:
            f.write("foo: bar\n")
            f.write("a-b-c: xyz\n")

        expected = {"foo": "bar", "a_b_c": "xyz"}

        data = config._load_config(self.config_path)
        self.assertThat(data, Equals(expected))

        data = config._click_load_config(self.config_path, "ignored")
        self.assertThat(data, Equals(expected))

    def test_load_invalid_config(self):
        dir_path = os.path.dirname(self.config_path)
        os.makedirs(dir_path)
        with open(self.config_path, "w") as f:
            # Note the missing newlines...
            f.write("foo: bar")
            f.write("a-b-c: xyz")

        self.assertRaises(YamlValidationError, config._load_config, self.config_path)

    def test_missing_config(self):
        expected = dict()
        data = config._load_config(self.config_path)
        self.assertThat(data, Equals(expected))
