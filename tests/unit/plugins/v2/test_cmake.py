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

from testtools.matchers import Equals
from testtools import TestCase

from snapcraft.plugins.v2.cmake import CMakePlugin


class CMakePluginTest(TestCase):
    def test_schema(self):
        schema = CMakePlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "cmake-parameters": {
                            "type": "array",
                            "uniqueItems": True,
                            "items": {"type": "string"},
                            "default": [],
                        }
                    },
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = CMakePlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals({"gcc", "cmake"}))

    def test_get_build_environment(self):
        plugin = CMakePlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_environment(), Equals(dict()))

    def test_get_build_commands(self):
        class Options:
            cmake_parameters = list()

        plugin = CMakePlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    'cmake "${SNAPCRAFT_PART_SRC}"',
                    'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
                    'cmake --build . --target install -- DESTDIR="${SNAPCRAFT_PART_INSTALL}"',
                ]
            ),
        )

    def test_get_build_commands_with_cmake_parameters(self):
        class Options:
            cmake_parameters = [
                "-DVERBOSE=1",
                "-DCMAKE_INSTALL_PREFIX=/foo",
                '-DCMAKE_SPACED_ARGS="foo bar"',
                '-DCMAKE_USING_ENV="$SNAPCRAFT_PART_INSTALL"/bar',
            ]

        plugin = CMakePlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    'cmake "${SNAPCRAFT_PART_SRC}" '
                    "-DVERBOSE=1 "
                    "-DCMAKE_INSTALL_PREFIX=/foo "
                    '-DCMAKE_SPACED_ARGS="foo bar" '
                    '-DCMAKE_USING_ENV="$SNAPCRAFT_PART_INSTALL"/bar',
                    'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
                    'cmake --build . --target install -- DESTDIR="${SNAPCRAFT_PART_INSTALL}"',
                ]
            ),
        )
