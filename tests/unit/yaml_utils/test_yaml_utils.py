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

import io
import os

from testtools.matchers import Equals

from snapcraft import yaml_utils
from snapcraft.yaml_utils import YamlValidationError
from tests import unit


class YamlLoadTests(unit.TestCase):
    def test_load_yaml_file(self):
        path = os.path.join(self.path, "test.yaml")
        with open(path, "w") as f:
            f.write("foo: bar")

        cfg = yaml_utils.load_yaml_file(path)

        self.assertThat(cfg, Equals({"foo": "bar"}))

    def test_load_yaml_file_bad_yaml(self):
        path = os.path.join(self.path, "test.yaml")
        with open(path, "w") as f:
            # Note the missing newlines...
            f.write("foo: bar")
            f.write("a-b-c: xyz")

        self.assertRaises(YamlValidationError, yaml_utils.load_yaml_file, path)


class OctIntTest(unit.TestCase):
    def test_octint_dump(self):
        output = io.StringIO()
        yaml_utils.dump(dict(number=yaml_utils.OctInt(8)), stream=output)
        output.seek(0)

        self.assertThat(output.read().strip(), Equals("number: 0010"))
