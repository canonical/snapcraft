# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import snapcraft.yaml_utils.errors
from snapcraft.internal import errors
from tests import fixture_setup

from . import CommandBaseTestCase


class CleanbuildCase(CommandBaseTestCase):
    def test_cleanbuild_raises_when_yaml_is_valid(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, name="cleanbuild")
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        self.useFixture(snapcraft_yaml)

        self.assertRaises(
            errors.SnapcraftEnvironmentError, self.run_command, ["cleanbuild"]
        )

    def test_cleanbuild_raises_when_yaml_is_invalid(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, name="cleanbuild")
        snapcraft_yaml.update_part("part1", dict(sources="boo"))
        self.useFixture(snapcraft_yaml)

        self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            self.run_command,
            ["cleanbuild"],
        )
