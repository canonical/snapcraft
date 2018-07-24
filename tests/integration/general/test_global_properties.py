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
import os

import testscenarios

from tests import integration, fixture_setup


class UnicodePropertyTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        ("summary", dict(name="foo", summary="bar💩", description="baz")),
        ("description", dict(name="foo", summary="bar", description="baz💩")),
    ]

    def test_invalid_unicode_workaround(self):
        if not (
            os.getenv("SNAPCRAFT_FROM_SNAP", False)
            or os.getenv("SNAPCRAFT_FROM_DEB", False)
        ):
            self.skipTest(
                "The yaml unicode patch is applied to the snap "
                "and python3-yaml package, but not PyYAML in PyPI"
            )

        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path,
            name=self.name,
            summary=self.summary,
            description=self.description,
        )
        snapcraft_yaml.update_part("my-part", {"plugin": "nil"})
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft("pull")
