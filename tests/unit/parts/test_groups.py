# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import pytest

from snapcraft import preprocessors
from snapcraft_legacy.yaml_utils.errors import YamlValidationError


def test_file_without_groups():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {"plugin": "nil", "source": "no source"},
            "part2": {"plugin": "meson", "source": "no source", "after": ["part1"]},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "part2"],
            },
        },
    }

    transformed_data = preprocessors.preprocessor(data)
    assert data == transformed_data


def test_file_with_groups():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {"plugin": "nil", "source": "no source", "groups": ["binaries"]},
            "part2": {"plugin": "meson", "source": "no source", "groups": ["binaries"]},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "binaries"],
            },
        },
    }

    expected_data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {"plugin": "nil", "source": "no source"},
            "part2": {"plugin": "meson", "source": "no source"},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "part2"],
            },
        },
    }

    transformed_data = preprocessors.preprocessor(data)
    assert expected_data == transformed_data


def test_group_with_part_name():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {"plugin": "nil", "source": "no source", "groups": ["binaries"]},
            "part2": {"plugin": "meson", "source": "no source", "groups": ["part1"]},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "binaries"],
            },
        },
    }

    with pytest.raises(YamlValidationError):
        preprocessors.preprocessor(data)


def test_wrong_groups_type():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {"plugin": "nil", "source": "no source", "groups": "binaries"},
            "part2": {"plugin": "meson", "source": "no source", "groups": ["part1"]},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "binaries"],
            },
        },
    }

    with pytest.raises(YamlValidationError):
        preprocessors.preprocessor(data)


def test_wrong_group_content_type():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "A test yaml data",
        "description": "A long description",
        "parts": {
            "part1": {
                "plugin": "nil",
                "source": "no source",
                "groups": [1, "binaries"],
            },
            "part2": {"plugin": "meson", "source": "no source", "groups": ["part1"]},
            "part3": {
                "plugin": "cmake",
                "source": "no source",
                "after": ["part1", "binaries"],
            },
        },
    }

    with pytest.raises(YamlValidationError):
        preprocessors.preprocessor(data)
