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

import os
import sys

from testtools import TestCase
from testtools.matchers import Equals

import snapcraft.plugins.v2.crystal as crystal
from snapcraft.plugins.v2.crystal import CrystalPlugin


class CrystalPluginTest(TestCase):
    def test_schema(self):
        self.assertThat(
            CrystalPlugin.get_schema(),
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "additionalProperties": False,
                    "type": "object",
                    "properties": {
                        "crystal-channel": {
                            "type": "string",
                            "default": "latest/stable",
                        },
                        "crystal-build-options": {
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
            crystal_channel = "latest/edge"

        self.assertThat(
            CrystalPlugin(part_name="my-part", options=Options()).get_build_snaps(),
            Equals({"crystal/latest/edge"}),
        )

    def test_get_build_packages(self):
        self.assertThat(
            CrystalPlugin(
                part_name="my-part", options=lambda: None
            ).get_build_packages(),
            Equals(
                {
                    "git",
                    "make",
                    "gcc",
                    "pkg-config",
                    "libssl-dev",
                    "libxml2-dev",
                    "libyaml-dev",
                    "libgmp-dev",
                    "libpcre3-dev",
                    "libevent-dev",
                    "libz-dev",
                }
            ),
        )

    def test_get_build_environment(self):
        self.assertThat(
            CrystalPlugin(
                part_name="my-part", options=lambda: None
            ).get_build_environment(),
            Equals(dict()),
        )


def test_get_build_commands(monkeypatch):
    class Options:
        crystal_channel = "latest/stable"
        crystal_build_options = []

    monkeypatch.setattr(sys, "path", ["", "/test"])
    monkeypatch.setattr(sys, "executable", "/test/python3")
    monkeypatch.setattr(crystal, "__file__", "/test/crystal.py")
    monkeypatch.setattr(os, "environ", dict())

    plugin = CrystalPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        "shards build --without-development ",
        'cp -r ./bin "${SNAPCRAFT_PART_INSTALL}"/bin',
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I /test/crystal.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC}" --part-install '
        '"${SNAPCRAFT_PART_INSTALL}" --part-build "${SNAPCRAFT_PART_BUILD}" '
        '--arch-triplet "${SNAPCRAFT_ARCH_TRIPLET}" --content-dirs '
        '"${SNAPCRAFT_CONTENT_DIRS}"',
    ]


def test_get_build_commands_with_build_options(monkeypatch):
    class Options:
        crystal_channel = "latest/stable"
        crystal_build_options = [
            "--release",
            "--static",
            "--link-flags=-s -wl,-z,relro,-z,now",
        ]

    monkeypatch.setattr(sys, "path", ["", "/test"])
    monkeypatch.setattr(sys, "executable", "/test/python3")
    monkeypatch.setattr(crystal, "__file__", "/test/crystal.py")
    monkeypatch.setattr(os, "environ", dict())

    plugin = CrystalPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        "shards build --without-development --release --static '--link-flags=-s -wl,-z,relro,-z,now'",
        'cp -r ./bin "${SNAPCRAFT_PART_INSTALL}"/bin',
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I /test/crystal.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC}" --part-install '
        '"${SNAPCRAFT_PART_INSTALL}" --part-build "${SNAPCRAFT_PART_BUILD}" '
        '--arch-triplet "${SNAPCRAFT_ARCH_TRIPLET}" --content-dirs '
        '"${SNAPCRAFT_CONTENT_DIRS}"',
    ]
