# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020-2021 Canonical Ltd
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

from snapcraft.plugins.v2.cmake import CMakePlugin


def test_schema():
    assert CMakePlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "cmake-parameters": {
                "type": "array",
                "uniqueItems": True,
                "items": {"type": "string"},
                "default": [],
            },
            "cmake-generator": {
                "type": "string",
                "enum": ["Unix Makefiles", "Ninja"],
                "default": "Unix Makefiles",
            },
        },
    }


def test_get_build_packages():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Unix Makefiles"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_packages() == {"gcc", "cmake"}


def test_get_build_packages_ninja():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Ninja"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_packages() == {"gcc", "cmake", "ninja-build"}


def test_get_build_environment():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Unix Makefiles"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_environment() == {
        "CMAKE_PREFIX_PATH": "${SNAPCRAFT_STAGE}",
        "SNAPCRAFT_CMAKE_ARGS": "",
    }


def test_get_build_commands():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Unix Makefiles"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'cmake "${SNAPCRAFT_PART_SRC_WORK}" -G "Unix Makefiles" ${SNAPCRAFT_CMAKE_ARGS}',
        'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'DESTDIR="${SNAPCRAFT_PART_INSTALL}" cmake --build . --target install',
    ]


def test_get_build_commands_ninja():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Ninja"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'cmake "${SNAPCRAFT_PART_SRC_WORK}" -G "Ninja" ${SNAPCRAFT_CMAKE_ARGS}',
        'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'DESTDIR="${SNAPCRAFT_PART_INSTALL}" cmake --build . --target install',
    ]


def test_get_build_commands_with_cmake_parameters():
    class Options:
        cmake_parameters = [
            "-DVERBOSE=1",
            "-DCMAKE_INSTALL_PREFIX=/foo",
            '-DCMAKE_SPACED_ARGS="foo bar"',
            '-DCMAKE_USING_ENV="$SNAPCRAFT_PART_INSTALL"/bar',
        ]
        cmake_generator = "Unix Makefiles"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'cmake "${SNAPCRAFT_PART_SRC_WORK}" -G "Unix Makefiles" '
        "${SNAPCRAFT_CMAKE_ARGS} "
        "-DVERBOSE=1 "
        "-DCMAKE_INSTALL_PREFIX=/foo "
        '-DCMAKE_SPACED_ARGS="foo bar" '
        '-DCMAKE_USING_ENV="$SNAPCRAFT_PART_INSTALL"/bar',
        'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        'DESTDIR="${SNAPCRAFT_PART_INSTALL}" cmake --build . --target install',
    ]


def test_out_of_source_build():
    class Options:
        cmake_parameters = list()
        cmake_generator = "Unix Makefiles"

    plugin = CMakePlugin(part_name="my-part", options=Options())

    assert plugin.out_of_source_build is True
