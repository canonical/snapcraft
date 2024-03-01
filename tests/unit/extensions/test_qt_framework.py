# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

from snapcraft.extensions import qt_framework
from snapcraft.extensions.qt_framework import _CONTENT_SNAP

############
# Fixtures #
############

fixture_variables = "name,yaml_data,arch,target_arch"
base_values = [
    (
        "qt6-6",
        {"base": "core22", "parts": {}},
        "amd64",
        "amd64",
    ),
    (
        "qt6-5",
        {"base": "core22", "parts": {}},
        "amd64",
        "amd64",
    ),
    (
        "qt5-15",
        {"base": "core22", "parts": {}},
        "amd64",
        "amd64",
    ),
]

builtin_stable_values = [
    (
        "qt6-6",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/stable"]}},
        },
        "amd64",
        "amd64",
    ),
    (
        "qt6-5",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/stable"]}},
        },
        "amd64",
        "amd64",
    ),
    (
        "qt5-15",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/5.15/stable"]}},
        },
        "amd64",
        "amd64",
    ),
]

builtin_edge_values = [
    (
        "qt6-6",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/edge"]}},
        },
        "amd64",
        "amd64",
    ),
    (
        "qt6-5",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/edge"]}},
        },
        "amd64",
        "amd64",
    ),
    (
        "qt5-15",
        {
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/5.15/edge"]}},
        },
        "amd64",
        "amd64",
    ),
]


@pytest.fixture
def qt_framework_extension(name, yaml_data, arch, target_arch):
    return qt_framework.QTFramework(
        name=name,
        yaml_data=yaml_data,
        arch=arch,
        target_arch=target_arch,
    )


#########################
# QTFramework Extension #
#########################


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_supported_bases(qt_framework_extension):
    assert qt_framework_extension.get_supported_bases() == ("core22",)


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_supported_confinement(qt_framework_extension):
    assert qt_framework_extension.get_supported_confinement() == ("strict", "devmode")


@pytest.mark.parametrize(fixture_variables, base_values)
def test_is_experimental(qt_framework_extension):
    assert qt_framework_extension.is_experimental(base="core22") is False


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_app_snippet(qt_framework_extension):
    assert qt_framework_extension.get_app_snippet() == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
        "environment": {
            "QT_PLUGIN_PATH": "$SNAP/qt-framework/usr/plugins:$SNAP/usr/lib/plugins",
        },
    }


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_root_snippet(qt_framework_extension, name, yaml_data):
    assert qt_framework_extension.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "compression": "lzo",
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/qt-framework"},
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
            }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/qt-framework/usr/share/X11"}},
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "icon-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
                "default-provider": "gtk-common-themes",
            },
            "sound-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
                "default-provider": "gtk-common-themes",
            },
            f"qt-framework-{qt_framework_extension.name[2:]}-core22": {
                "interface": "content",
                "default-provider": _CONTENT_SNAP[name][yaml_data["base"]],
                "target": "$SNAP/qt-framework",
            },
        },
    }


@pytest.mark.parametrize(fixture_variables, builtin_stable_values)
def test_get_root_snippet_with_external_sdk(qt_framework_extension, name, yaml_data):
    assert qt_framework_extension.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "compression": "lzo",
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/qt-framework"},
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
            }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/qt-framework/usr/share/X11"}},
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "icon-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
                "default-provider": "gtk-common-themes",
            },
            "sound-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
                "default-provider": "gtk-common-themes",
            },
            f"qt-framework-{qt_framework_extension.name[2:]}-core22": {
                "interface": "content",
                "default-provider": _CONTENT_SNAP[name][yaml_data["base"]],
                "target": "$SNAP/qt-framework",
            },
        },
    }


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_part_snippet(qt_framework_extension):
    assert qt_framework_extension.get_part_snippet(plugin_name="cmake") == {
        "build-environment": [
            {"PATH": "/snap/qt-framework-sdk/current/usr/bin${PATH:+:$PATH}"},
            {
                "XDG_DATA_DIRS": "$CRAFT_STAGE/usr/share:"
                "/snap/qt-framework-sdk/current/usr/share:"
                "/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
            },
            {
                "SNAPCRAFT_CMAKE_ARGS": "-DCMAKE_FIND_ROOT_PATH=/snap/qt-framework-sdk/current:"
                "-DCMAKE_PREFIX_PATH=/snap/qt-framework-sdk/current/usr:"
                "-DZLIB_INCLUDE_DIR=/lib/x86_64-linux-gnu"
                "${SNAPCRAFT_CMAKE_ARGS:+:$SNAPCRAFT_CMAKE_ARGS}"
            },
        ],
        "build-packages": [
            "libgl1-mesa-dev",
            "libpcre2-16-0",
            "libglib2.0-0",
            "libdouble-conversion3",
            "libb2-1",
        ],
        "stage-packages": [
            "libpcre2-16-0",
            "libglib2.0-0",
            "libdouble-conversion3",
            "libb2-1",
        ],
    }


@pytest.mark.parametrize(fixture_variables, base_values)
def test_get_parts_snippet_without_external_sdk(qt_framework_extension, name):
    assert qt_framework_extension.get_parts_snippet() == {}


@pytest.mark.parametrize(fixture_variables, builtin_stable_values)
def test_get_parts_snippet_with_external_sdk(qt_framework_extension, name):
    sdk_snap = qt_framework_extension.qt_snaps.sdk["snap"]
    sdk_channel = qt_framework_extension.qt_snaps.sdk["channel"]

    assert qt_framework_extension.get_parts_snippet() == {
        f"{name}/sdk": {"plugin": "nil", "build-snaps": [f"{sdk_snap}/{sdk_channel}"]}
    }


@pytest.mark.parametrize(fixture_variables, builtin_edge_values)
def test_get_parts_snippet_with_external_sdk_different_channel(
    qt_framework_extension, name
):
    sdk_snap = qt_framework_extension.qt_snaps.sdk["snap"]
    sdk_channel = qt_framework_extension.qt_snaps.sdk["channel"]

    assert qt_framework_extension.get_parts_snippet() == {
        f"{name}/sdk": {"plugin": "nil", "build-snaps": [f"{sdk_snap}/{sdk_channel}"]}
    }
