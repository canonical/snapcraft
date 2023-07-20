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
from snapcraft.extensions.extension import get_extensions_data_dir

_EXTENSION_NAME = "qt6-5"

############
# Fixtures #
############


@pytest.fixture
def qt_framework_extension():
    return qt_framework.QTFramework(
        name=_EXTENSION_NAME,
        yaml_data={"base": "core22", "parts": {}},
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def qt_framework_extension_with_build_snap():
    return qt_framework.QTFramework(
        name=_EXTENSION_NAME,
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/stable"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def qt_framework_extension_with_default_build_snap_from_latest_edge():
    return qt_framework.QTFramework(
        name=_EXTENSION_NAME,
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["qt-framework-sdk/6.5/edge"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# QTFramework Extension #
###################


def test_get_supported_bases(qt_framework_extension):
    assert qt_framework_extension.get_supported_bases() == ("core22",)


def test_get_supported_confinement(qt_framework_extension):
    assert qt_framework_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert qt_framework.QTFramework.is_experimental(base="core22") is False


def test_get_app_snippet(qt_framework_extension):
    assert qt_framework_extension.get_app_snippet() == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
        "environment": {
            "QT_PLUGIN_PATH": "$SNAP/qt-framework/usr/plugins:$SNAP/usr/lib/plugins",
        },
    }


def test_get_root_snippet(qt_framework_extension):
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
            "qt-framework": {
                "interface": "content",
                "default-provider": "qt-framework-6-5-core22",
                "target": "$SNAP/qt-framework",
            },
        },
    }


def test_get_root_snippet_with_external_sdk(qt_framework_extension_with_build_snap):
    assert qt_framework_extension_with_build_snap.get_root_snippet() == {
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
            "qt-framework": {
                "interface": "content",
                "default-provider": "qt-framework-6-5-core22",
                "target": "$SNAP/qt-framework",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for QTFramework.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, qt_framework_extension):
        self.assert_get_part_snippet(qt_framework_extension)

    def test_get_part_snippet_latest_edge(
        self, qt_framework_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            qt_framework_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(qt_framework_instance):
        assert qt_framework_instance.get_part_snippet(plugin_name="cmake") == {
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
            "build-packages": ["libgl1-mesa-dev"],
        }


def test_get_part_snippet_with_external_sdk(qt_framework_extension_with_build_snap):
    assert qt_framework_extension_with_build_snap.get_part_snippet(
        plugin_name="cmake"
    ) == {
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
        "build-packages": ["libgl1-mesa-dev"],
    }


def test_get_parts_snippet(qt_framework_extension):
    source = get_extensions_data_dir() / "desktop" / "command-chain"

    assert qt_framework_extension.get_parts_snippet() == {
        "qt6-5/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=qt-framework"],
            "build-snaps": ["qt-framework-sdk/6.5/stable"],
        }
    }


def test_get_parts_snippet_with_external_sdk(qt_framework_extension_with_build_snap):
    source = get_extensions_data_dir() / "desktop" / "command-chain"

    assert qt_framework_extension_with_build_snap.get_parts_snippet() == {
        "qt6-5/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=qt-framework"],
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    qt_framework_extension_with_default_build_snap_from_latest_edge,
):
    source = get_extensions_data_dir() / "desktop" / "command-chain"
    assert qt_framework_extension_with_default_build_snap_from_latest_edge.get_parts_snippet() == {
        "qt6-5/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=qt-framework"],
        }
    }
