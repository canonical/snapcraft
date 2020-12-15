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
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.autotools import AutotoolsPlugin


class AutotoolsPluginTest(TestCase):
    def test_schema(self):
        schema = AutotoolsPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "autotools-configure-parameters": {
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
        plugin = AutotoolsPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_packages(),
            Equals({"autoconf", "automake", "autopoint", "gcc", "libtool"}),
        )

    def test_get_build_environment(self):
        plugin = AutotoolsPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_environment(), Equals(dict()))

    def test_get_build_commands(self):
        class Options:
            autotools_configure_parameters = list()

        plugin = AutotoolsPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "[ ! -f ./configure ] && autoreconf --install",
                    "./configure",
                    'make -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
                    'make install DESTDIR="${SNAPCRAFT_PART_INSTALL}"',
                ]
            ),
        )

    def test_get_build_commands_with_configure_parameters(self):
        class Options:
            autotools_configure_parameters = ["--with-foo=true", "--prefix=/foo"]

        plugin = AutotoolsPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "[ ! -f ./configure ] && autoreconf --install",
                    "./configure --with-foo=true --prefix=/foo",
                    'make -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
                    'make install DESTDIR="${SNAPCRAFT_PART_INSTALL}"',
                ]
            ),
        )
