# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from textwrap import dedent

import fixtures
from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.npm import NpmPlugin


class NpmPluginTest(TestCase):
    def test_schema(self):
        schema = NpmPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "additionalProperties": False,
                    "properties": {"npm-node-version": {"type": "string"}},
                    "required": ["source", "npm-node-version"],
                    "type": "object",
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = NpmPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals({"curl", "gcc"}))

    def test_get_build_environment(self):
        plugin = NpmPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals({"PATH": "${SNAPCRAFT_PART_INSTALL}/bin:${PATH}"}),
        )

    def test_get_architecture_from_snap_arch(self):
        for snap_arch, node_arch in [
            ("amd64", "x64"),
            ("i386", "x86"),
            ("armhf", "armv7l"),
            ("arm64", "arm64"),
            ("ppc64el", "ppc64le"),
            ("s390x", "s390x"),
        ]:
            self.useFixture(fixtures.EnvironmentVariable("SNAP_ARCH", snap_arch))
            self.expectThat(NpmPlugin._get_architecture(), Equals(node_arch))

    def test_get_architecture_from_platform_for_x86(self):
        for platform_architecture, node_arch in [("32bit", "x86"), ("64bit", "x64")]:
            self.useFixture(
                fixtures.MockPatch("platform.machine", return_value="x86_64")
            )
            self.useFixture(
                fixtures.MockPatch(
                    "platform.architecture", return_value=("64bit", "ELF")
                )
            )
            self.expectThat(NpmPlugin._get_architecture(), Equals("x64"))

    def test_get_build_commands(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAP_ARCH", "amd64"))

        class Options:
            npm_node_version = "6.0.0"

        plugin = NpmPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    dedent(
                        """\
                    if [ ! -f "${SNAPCRAFT_PART_INSTALL}/bin/node" ]; then
                        curl -s "https://nodejs.org/dist/v6.0.0/node-v6.0.0-linux-x64.tar.gz" | tar xzf - -C "${SNAPCRAFT_PART_INSTALL}/" --strip-components=1
                    fi
                    """
                    ),
                    'npm install -g --prefix "${SNAPCRAFT_PART_INSTALL}" $(npm pack . | tail -1)',
                ]
            ),
        )
