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

from snapcraft.plugins.v2.qmake import QMakePlugin


def test_schema():
    assert QMakePlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "qmake-parameters": {
                "type": "array",
                "uniqueItems": True,
                "items": {"type": "string"},
                "default": [],
            },
            "qmake-project-file": {"type": "string", "default": ""},
        },
    }


def test_get_build_packages():
    class Options:
        qmake_parameters = list()
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_packages() == {"g++", "make", "qt5-qmake"}


def test_get_build_snaps():
    class Options:
        qmake_parameters = list()
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_snaps() == set()


def test_get_build_environment():
    class Options:
        qmake_parameters = list()
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_environment() == {"QT_SELECT": "qt5"}


def test_out_of_source_build():
    class Options:
        qmake_parameters = list()
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.out_of_source_build is True


def test_get_build_commands():
    class Options:
        qmake_parameters = list()
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'qmake QMAKE_CFLAGS+="${CFLAGS:-}" QMAKE_CXXFLAGS+="${CXXFLAGS:-}" QMAKE_LFLAGS+="${LDFLAGS:-}" "${SNAPCRAFT_PART_SRC_WORK}"',
        'env -u CFLAGS -u CXXFLAGS make -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'make install INSTALL_ROOT="${SNAPCRAFT_PART_INSTALL}"',
    ]


def test_get_build_commands_with_qmake_parameters():
    class Options:
        qmake_parameters = ["-Wall"]
        qmake_project_file = ""

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'qmake QMAKE_CFLAGS+="${CFLAGS:-}" QMAKE_CXXFLAGS+="${CXXFLAGS:-}" QMAKE_LFLAGS+="${LDFLAGS:-}" -Wall "${SNAPCRAFT_PART_SRC_WORK}"',
        'env -u CFLAGS -u CXXFLAGS make -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'make install INSTALL_ROOT="${SNAPCRAFT_PART_INSTALL}"',
    ]


def test_get_build_commands_with_qmake_project_file():
    class Options:
        qmake_parameters = list()
        qmake_project_file = "foo.pro"

    plugin = QMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'qmake QMAKE_CFLAGS+="${CFLAGS:-}" QMAKE_CXXFLAGS+="${CXXFLAGS:-}" QMAKE_LFLAGS+="${LDFLAGS:-}" "${SNAPCRAFT_PART_SRC_WORK}/foo.pro"',
        'env -u CFLAGS -u CXXFLAGS make -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'make install INSTALL_ROOT="${SNAPCRAFT_PART_INSTALL}"',
    ]
