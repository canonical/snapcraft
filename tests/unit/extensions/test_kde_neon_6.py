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

from snapcraft.extensions import kde_neon_6
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def kde_neon_6_extension():
    return kde_neon_6.KDENeon(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_6_extension_with_build_snap():
    return kde_neon_6.KDENeon(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": ["kde-qt6-core22-sdk/latest/stable", "kf6-core22-sdk/latest/stable"]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_6_extension_with_default_build_snap_from_latest_edge():
    return kde_neon_6.KDENeon(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": ["kde-qt6-core22-sdk/latest/edge", "kf6-core22-sdk/latest/edge"]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# KDENeon Extension #
###################


def test_get_supported_bases(kde_neon_6_extension):
    assert kde_neon_6_extension.get_supported_bases() == ("core22",)


def test_get_supported_confinement(kde_neon_6_extension):
    assert kde_neon_6_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert kde_neon_6.KDENeon.is_experimental(base="core22") is False


def test_get_app_snippet(kde_neon_6_extension):
    assert kde_neon_6_extension.get_app_snippet() == {
        "command-chain": ["snap/command-chain/desktop-launch6"],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11", "audio-playback",
                  "unity7", "network", "network-bind"],
    }


def test_get_root_snippet(kde_neon_6_extension):
    assert kde_neon_6_extension.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf6"},
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf6/usr/share/X11"}},
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
            "kde-qt6-core22": {
                "content": "kde-qt6-core22-all",
                "interface": "content",
                "default-provider": "kde-qt6-core22",
                "target": "$SNAP/kf6",
            },
            "kf6-core22": {
                "content": "kf6-core22-all",
                "interface": "content",
                "default-provider": "kf6-core22",
                "target": "$SNAP/kf6",
            },
        },
    }


def test_get_root_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    assert kde_neon_6_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf6"},
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf6/usr/share/X11"}},
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
            "kde-qt6-core22": {
                "content": "kde-qt6-core22-all",
                "interface": "content",
                "default-provider": "kde-qt6-core22",
                "target": "$SNAP/kf6",
            },
            "kf6-core22": {
                "content": "kf6-core22-all",
                "interface": "content",
                "default-provider": "kf6-core22",
                "target": "$SNAP/kf6",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for KDENeon.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, kde_neon_6_extension):
        self.assert_get_part_snippet(kde_neon_6_extension)

    def test_get_part_snippet_latest_edge(
        self, kde_neon_6_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            kde_neon_6_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(kde_neon_6_instance):
        assert kde_neon_6_instance.get_part_snippet(plugin_name="cmake") == {
            "build-environment": [
                {
                    "PATH": (
                        "/snap/kde-qt6-core22-sdk/current/usr/bin:/snap/kf6-core22-sdk/current/usr/bin${PATH:+:$PATH}"
                    )
                },
                {
                    "XDG_DATA_DIRS": (
                       "$CRAFT_STAGE/usr/share:/snap/kde-qt6-core22-sdk/current/usr/share:"
                        "/snap/kf6-core22-sdk/current/usr/share:"
                        "/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "SNAPCRAFT_CMAKE_ARGS": (
                        "-DCMAKE_FIND_ROOT_PATH="
                        "/snap/kde-qt6-core22-sdk/current;"
                        "/snap/kf6-core22-sdk/current${SNAPCRAFT_CMAKE_ARGS:+:$SNAPCRAFT_CMAKE_ARGS}"
                    )
                },
            ]
        }


def test_get_part_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    assert kde_neon_6_extension_with_build_snap.get_part_snippet(plugin_name="cmake") == {
        "build-environment": [
                {
                    "PATH": (
                        "/snap/kde-qt6-core22-sdk/current/usr/bin:/snap/kf6-core22-sdk/current/usr/bin${PATH:+:$PATH}"
                    )
                },
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:/snap/kde-qt6-core22-sdk/current/usr/share:"
                        "/snap/kf6-core22-sdk/current/usr/share:"
                        "/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "SNAPCRAFT_CMAKE_ARGS": (
                        "-DCMAKE_FIND_ROOT_PATH="
                        "/snap/kde-qt6-core22-sdk/current;"
                        "/snap/kf6-core22-sdk/current${SNAPCRAFT_CMAKE_ARGS:+:$SNAPCRAFT_CMAKE_ARGS}"
                    )
                },
        ]
    }


def test_get_parts_snippet(kde_neon_6_extension):
    source = get_extensions_data_dir() / "desktop" / "kde-neon-6"

    assert kde_neon_6_extension.get_parts_snippet() == {
        "kde-neon-6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=kde-qt6-core22, kf6-core22"],
            "build-snaps": ["kde-qt6-core22-sdk", "kf6-core22-sdk"],
        }
    }


def test_get_parts_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    source = get_extensions_data_dir() / "desktop" / "kde-neon-6"

    assert kde_neon_6_extension_with_build_snap.get_parts_snippet() == {
        "kde-neon-6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=kde-qt6-core22, kf6-core22"],
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    kde_neon_6_extension_with_default_build_snap_from_latest_edge,
):
    source = get_extensions_data_dir() / "desktop" / "kde-neon-6"
    assert (
        kde_neon_6_extension_with_default_build_snap_from_latest_edge.get_parts_snippet()
        == {
            "kde-neon-6/sdk": {
                "source": str(source),
                "plugin": "make",
                "make-parameters": ["PLATFORM_PLUG=kde-qt6-core22, kf6-core22"],
            }
        }
    )
