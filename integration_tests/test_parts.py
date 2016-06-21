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

import yaml
from xdg import BaseDirectory

import integration_tests


class PartsTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')
        self.headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')

    def test_update(self):
        self.run_snapcraft('update')

        self.assertTrue(os.path.exists(self.parts_yaml))
        self.assertTrue(os.path.exists(self.headers_yaml))

    def test_curl_exists(self):
        """Curl is used in most of the demos so we test for its existence."""
        self.run_snapcraft('update')

        with open(self.parts_yaml) as parts_file:
            parts = yaml.load(parts_file)

        self.assertTrue('curl' in parts, parts)
