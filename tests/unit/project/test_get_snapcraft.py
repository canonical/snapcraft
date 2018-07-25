# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.project import get_snapcraft_yaml, errors
from tests import unit


class GetSnapcraftYamlTest(unit.TestCase):

    scenarios = [
        ("snapcraft.yaml", dict(file_path="snapcraft.yaml")),
        ("snap/snapcraft.yaml", dict(file_path=os.path.join("snap", "snapcraft.yaml"))),
        (".snapcraft.yaml", dict(file_path=".snapcraft.yaml")),
    ]

    def test_get(self):
        if os.path.dirname(self.file_path):
            os.makedirs(os.path.dirname(self.file_path))
        open(self.file_path, "w").close()
        self.assertThat(get_snapcraft_yaml(), Equals(self.file_path))


class GetSnapcraftYamlMissingErrorsTest(unit.TestCase):
    def test_config_raises_on_missing_snapcraft_yaml(self):
        """Test that an error is raised if snap/snapcraft.yaml is missing"""

        self.assertRaises(errors.MissingSnapcraftYamlError, get_snapcraft_yaml)


class GetSnapcraftYamlDuplicateErrorsTest(unit.TestCase):

    scenarios = [
        (
            "snapcraft.yaml and .snapcraft.yaml",
            dict(file_path1="snapcraft.yaml", file_path2=".snapcraft.yaml"),
        ),
        (
            "snapcraft.yaml and snap/snapcraft.yaml",
            dict(
                file_path1="snapcraft.yaml",
                file_path2=os.path.join("snap", "snapcraft.yaml"),
            ),
        ),
        (
            ".snapcraft.yaml and snap/snapcraft.yaml",
            dict(
                file_path1=".snapcraft.yaml",
                file_path2=os.path.join("snap", "snapcraft.yaml"),
            ),
        ),
    ]

    def test_duplicates(self):
        if os.path.dirname(self.file_path1):
            os.makedirs(os.path.dirname(self.file_path1))
        open(self.file_path1, "w").close()

        if os.path.dirname(self.file_path2):
            os.makedirs(os.path.dirname(self.file_path2))
        open(self.file_path2, "w").close()

        self.assertRaises(errors.DuplicateSnapcraftYamlError, get_snapcraft_yaml)
