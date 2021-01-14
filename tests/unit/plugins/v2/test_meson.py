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

from snapcraft.plugins.v2.meson import MesonPlugin


class MesonPluginTest(TestCase):
    def test_schema(self):
        schema = MesonPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "additionalProperties": False,
                    "properties": {
                        "meson-parameters": {
                            "default": [],
                            "items": {"type": "string"},
                            "type": "array",
                            "uniqueItems": True,
                        },
                        "meson-version": {"default": "", "type": "string"},
                    },
                    "required": ["source"],
                    "type": "object",
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = MesonPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_packages(),
            Equals(
                {
                    "ninja-build",
                    "gcc",
                    "python3-pip",
                    "python3-setuptools",
                    "python3-wheel",
                }
            ),
        )

    def test_get_build_environment(self):
        plugin = MesonPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_environment(), Equals(dict()))

    def test_get_build_commands(self):
        class Options:
            meson_parameters = list()
            meson_version = ""

        plugin = MesonPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "/usr/bin/python3 -m pip install -U meson",
                    '[ ! -f build.ninja ] && meson "${SNAPCRAFT_PART_SRC_WORK}"',
                    "ninja",
                    'DESTDIR="${SNAPCRAFT_PART_INSTALL}" ninja install',
                ]
            ),
        )

    def test_get_build_commands_with_options(self):
        class Options:
            meson_parameters = ["--buildtype=release"]
            meson_version = "2.2"

        plugin = MesonPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "/usr/bin/python3 -m pip install -U meson==2.2",
                    '[ ! -f build.ninja ] && meson --buildtype=release "${SNAPCRAFT_PART_SRC_WORK}"',
                    "ninja",
                    'DESTDIR="${SNAPCRAFT_PART_INSTALL}" ninja install',
                ]
            ),
        )

    def test_out_of_source_build(self):
        plugin = MesonPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.out_of_source_build, Equals(True))
