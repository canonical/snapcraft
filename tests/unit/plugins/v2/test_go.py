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

from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.go import GoPlugin


class GoPluginTest(TestCase):
    def test_schema(self):
        schema = GoPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "go-channel": {"type": "string", "default": "latest/stable"},
                        "go-buildtags": {
                            "type": "array",
                            "uniqueItems": True,
                            "items": {"type": "string"},
                            "default": [],
                        },
                    },
                    "required": ["source"],
                }
            ),
        )

    def test_get_build_snaps(self):
        class Options:
            go_channel = "14/latest"

        plugin = GoPlugin(part_name="my-part", options=Options())

        self.assertThat(plugin.get_build_snaps(), Equals({"go/14/latest"}))

    def test_get_build_packages(self):
        plugin = GoPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals({"gcc"}))

    def test_get_build_environment(self):
        plugin = GoPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals(
                {
                    "CGO_ENABLED": "1",
                    "GOBIN": "${SNAPCRAFT_PART_INSTALL}/bin",
                    "SNAPCRAFT_GO_LDFLAGS": "-ldflags -linkmode=external",
                }
            ),
        )

    def test_get_build_commands(self):
        class Options:
            go_buildtags = list()

        plugin = GoPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "go mod download",
                    'go install -p "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"  ${SNAPCRAFT_GO_LDFLAGS} ./...',
                ]
            ),
        )

    def test_get_build_commands_with_buildtags(self):
        class Options:
            go_buildtags = ["dev", "debug"]

        plugin = GoPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "go mod download",
                    'go install -p "${SNAPCRAFT_PARALLEL_BUILD_COUNT}" -tags=dev,debug ${SNAPCRAFT_GO_LDFLAGS} ./...',
                ]
            ),
        )
