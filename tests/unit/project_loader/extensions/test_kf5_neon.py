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

import textwrap

from testtools.matchers import Equals
from .. import ProjectLoaderBaseTest


class Kf5NeonExtensionTest(ProjectLoaderBaseTest):
    def test_extension(self):
        project = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: test-command
                        adapter: full
                        extensions: [kf5-neon]

                parts:
                    part1:
                        plugin: nil
                """
            )
        )

        for app in project.data["apps"]:
            self.assertThat(project.data["apps"][app]["adapter"], Equals("full"))
            command_chain = project.data["apps"][app]["command-chain"]
            self.assertThat(command_chain, Equals([
                "snap/command-chain/kf5-connect",
                "snap/command-chain/desktop-init",
                "snap/command-chain/desktop-common",
                "snap/command-chain/kf5-launch",
            ]))

        all_parts = project.data["parts"]
        expected_parts = [
            "part1",
            "desktop-common-extension",
            "desktop-common-bindtextdomain",
            "kde-frameworks-5-env",
        ]
        for part in expected_parts:
            self.assertIn(part, all_parts)

        all_plugs = project.data["plugs"]
        expected_plugs = ["icon-themes", "sound-themes", "kde-frameworks-5-plug"]
        for plug in expected_plugs:
            self.assertIn(plug, all_plugs)

        self.assertThat(project.data["assumes"], Equals(["command-chain"]))
