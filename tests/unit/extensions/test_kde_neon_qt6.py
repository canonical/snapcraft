# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
# Copyright 2023-2025 Scarlett Moore <sgmoore@kde.org>
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

from snapcraft.extensions import kde_neon_qt6
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def kde_neon_qt6_extension():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_qt6_extension_core24():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={"base": "core24", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_qt6_extension_with_build_snap():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core22-sdk/latest/stable",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_qt6_extension_with_build_snap_core24():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={
            "base": "core24",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core24-sdk/latest/stable",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_qt6_extension_with_default_build_snap_from_latest_edge():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core22-sdk/latest/edge",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_qt6_extension_with_default_build_snap_from_latest_edge_core24():
    return kde_neon_qt6.KDENeonQt6(
        yaml_data={
            "base": "core24",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core24-sdk/latest/edge",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# KDENeonQt6 Extension #
###################


def test_get_supported_bases(kde_neon_qt6_extension):
    assert kde_neon_qt6_extension.get_supported_bases() == ("core22", "core24")


def test_get_supported_confinement(kde_neon_qt6_extension):
    assert kde_neon_qt6_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert kde_neon_qt6.KDENeonQt6.is_experimental(base="core22") is False


def test_get_app_snippet(kde_neon_qt6_extension):
    assert kde_neon_qt6_extension.get_app_snippet(app_name="test-app") == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": [
            "desktop",
            "desktop-legacy",
            "opengl",
            "wayland",
            "x11",
            "audio-playback",
            "unity7",
            "network",
            "network-bind",
        ],
    }


def test_get_app_snippet_core24(kde_neon_qt6_extension_core24):
    assert kde_neon_qt6_extension_core24.get_app_snippet(app_name="test-app") == {
        "command-chain": [
            "snap/command-chain/gpu-2404-wrapper",
            "snap/command-chain/desktop-launch",
        ],
        "plugs": [
            "desktop",
            "desktop-legacy",
            "opengl",
            "wayland",
            "x11",
            "audio-playback",
            "unity7",
            "network",
            "network-bind",
        ],
    }


def test_get_root_snippet(kde_neon_qt6_extension):
    assert kde_neon_qt6_extension.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/qt6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kde-qt6-core22",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"symlink": "$SNAP/qt6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/qt6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/kde-qt6-core22/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gtk-2-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
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
                "target": "$SNAP/qt6",
            },
        },
    }


def test_get_root_snippet_core24(kde_neon_qt6_extension_core24):
    assert kde_neon_qt6_extension_core24.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/qt6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kde-qt6-core24",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"symlink": "$SNAP/qt6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/qt6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/gpu-2404/libdrm"},
            "/usr/share/drirc.d": {"symlink": "$SNAP/gpu-2404/drirc.d"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gtk-2-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
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
            "kde-qt6-core24": {
                "content": "kde-qt6-core24-all",
                "interface": "content",
                "default-provider": "kde-qt6-core24",
                "target": "$SNAP/qt6",
            },
            "gpu-2404": {
                "default-provider": "mesa-2404",
                "interface": "content",
                "target": "$SNAP/gpu-2404",
            },
        },
    }


def test_get_root_snippet_with_gpu(kde_neon_qt6_extension_core24):
    snippet = kde_neon_qt6_extension_core24.get_root_snippet()
    assert snippet["plugs"]["gpu-2404"] == {
        "default-provider": "mesa-2404",
        "interface": "content",
        "target": "$SNAP/gpu-2404",
    }
    assert snippet["layout"]["/usr/share/libdrm"] == {
        "bind": "$SNAP/gpu-2404/libdrm",
    }
    assert snippet["layout"]["/usr/share/drirc.d"] == {
        "symlink": "$SNAP/gpu-2404/drirc.d",
    }


def test_get_root_snippet_without_gpu(kde_neon_qt6_extension):
    snippet = kde_neon_qt6_extension.get_root_snippet()
    assert snippet["layout"]["/usr/share/libdrm"] == {
        "bind": "$SNAP/kde-qt6-core22/usr/share/libdrm",
    }


def test_get_root_snippet_no_exceptions(kde_neon_qt6_extension):
    with pytest.raises(AssertionError):
        assert not kde_neon_qt6_extension.get_root_snippet()


def test_get_root_snippet_with_external_sdk(kde_neon_qt6_extension_with_build_snap):
    assert kde_neon_qt6_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/qt6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kde-qt6-core22",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"symlink": "$SNAP/qt6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/qt6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/kde-qt6-core22/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gtk-2-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
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
                "target": "$SNAP/qt6",
            },
        },
    }


def test_get_root_snippet_with_external_sdk_core24(
    kde_neon_qt6_extension_with_build_snap_core24,
):
    assert kde_neon_qt6_extension_with_build_snap_core24.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/qt6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kde-qt6-core24",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"symlink": "$SNAP/qt6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/qt6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/gpu-2404/libdrm"},
            "/usr/share/drirc.d": {"symlink": "$SNAP/gpu-2404/drirc.d"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gtk-2-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes",
            },
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
            "kde-qt6-core24": {
                "content": "kde-qt6-core24-all",
                "interface": "content",
                "default-provider": "kde-qt6-core24",
                "target": "$SNAP/qt6",
            },
            "gpu-2404": {
                "default-provider": "mesa-2404",
                "interface": "content",
                "target": "$SNAP/gpu-2404",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for KDENeonQt6.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, kde_neon_qt6_extension):
        self.assert_get_part_snippet(kde_neon_qt6_extension)

    def test_get_part_snippet_latest_edge(
        self, kde_neon_qt6_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            kde_neon_qt6_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(kde_neon_qt6_instance):
        assert kde_neon_qt6_instance.get_part_snippet(plugin_name="cmake") == {
            "build-environment": [
                {
                    "PATH": (
                        "$CRAFT_STAGE/usr/bin:"
                        "/snap/kde-qt6-core22-sdk/current/usr/bin"
                        "${PATH:+:$PATH}"
                    )
                },
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:"
                        "/snap/kde-qt6-core22-sdk/current/usr/share"
                        "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "XDG_CONFIG_HOME": (
                        "$CRAFT_STAGE/etc/xdg:"
                        "/snap/kde-qt6-core22-sdk/current/etc/xdg"
                        "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                    )
                },
                {
                    "LD_LIBRARY_PATH": (
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/blas:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/lapack:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/libproxy:"
                        "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "$CRAFT_STAGE/usr/lib:"
                        "$CRAFT_STAGE/lib"
                        "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                    )
                },
                {
                    "CMAKE_PREFIX_PATH": (
                        "$CRAFT_STAGE/usr:"
                        "/snap/kde-qt6-core22-sdk/current/usr"
                        "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                    )
                },
            ]
        }


def test_get_part_snippet_with_external_sdk(kde_neon_qt6_extension_with_build_snap):
    assert kde_neon_qt6_extension_with_build_snap.get_part_snippet(
        plugin_name="cmake"
    ) == {
        "build-environment": [
            {
                "PATH": (
                    "$CRAFT_STAGE/usr/bin:"
                    "/snap/kde-qt6-core22-sdk/current/usr/bin"
                    "${PATH:+:$PATH}"
                )
            },
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:"
                    "/snap/kde-qt6-core22-sdk/current/usr/share"
                    "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "XDG_CONFIG_HOME": (
                    "$CRAFT_STAGE/etc/xdg:"
                    "/snap/kde-qt6-core22-sdk/current/etc/xdg"
                    "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                )
            },
            {
                "LD_LIBRARY_PATH": (
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/blas:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/lapack:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/libproxy:"
                    "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "$CRAFT_STAGE/usr/lib:"
                    "$CRAFT_STAGE/lib"
                    "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                )
            },
            {
                "CMAKE_PREFIX_PATH": (
                    "$CRAFT_STAGE/usr:"
                    "/snap/kde-qt6-core22-sdk/current/usr"
                    "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                )
            },
        ]
    }

    def test_get_part_snippet_core24(self, kde_neon_qt6_extension_core24):
        self.assert_get_part_snippet(kde_neon_qt6_extension_core24)


def assert_get_part_snippet(kde_neon_qt6_instance):
    assert kde_neon_qt6_instance.get_part_snippet(plugin_name="cmake") == {
        "build-environment": [
            {
                "PATH": (
                    "$CRAFT_STAGE/usr/bin:"
                    "/snap/kde-qt6-core24-sdk/current/usr/bin"
                    "${PATH:+:$PATH}"
                )
            },
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:"
                    "/snap/kde-qt6-core24-sdk/current/usr/share"
                    "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "XDG_CONFIG_HOME": (
                    "$CRAFT_STAGE/etc/xdg:"
                    "/snap/kde-qt6-core24-sdk/current/etc/xdg"
                    "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                )
            },
            {
                "LD_LIBRARY_PATH": (
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib:"
                    "/snap/mesa-2404/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/blas:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/lapack:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/libproxy:"
                    "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "$CRAFT_STAGE/usr/lib:"
                    "$CRAFT_STAGE/lib"
                    "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                )
            },
            {
                "CMAKE_PREFIX_PATH": (
                    "$CRAFT_STAGE/usr:"
                    "/snap/kde-qt6-core24-sdk/current/usr"
                    "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                )
            },
        ]
    }


def test_get_parts_snippet(kde_neon_qt6_extension):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_qt6_extension.get_parts_snippet() == {
        "kde-neon-qt6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "PLATFORM_PLUG=kde-qt6-core22",
            ],
            "build-snaps": ["kde-qt6-core22-sdk"],
        }
    }


def test_get_parts_snippet_core24(kde_neon_qt6_extension_core24):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_qt6_extension_core24.get_parts_snippet() == {
        "kde-neon-qt6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "GPU_WRAPPER=gpu-2404-wrapper",
                "PLATFORM_PLUG=kde-qt6-core24",
            ],
            "build-snaps": ["kde-qt6-core24-sdk"],
        }
    }


def test_get_parts_snippet_with_external_sdk(kde_neon_qt6_extension_with_build_snap):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_qt6_extension_with_build_snap.get_parts_snippet() == {
        "kde-neon-qt6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "PLATFORM_PLUG=kde-qt6-core22",
            ],
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    kde_neon_qt6_extension_with_default_build_snap_from_latest_edge_core24,
):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"
    assert (
        kde_neon_qt6_extension_with_default_build_snap_from_latest_edge_core24.get_parts_snippet()
        == {
            "kde-neon-qt6/sdk": {
                "source": str(source),
                "plugin": "make",
                "make-parameters": [
                    "GPU_WRAPPER=gpu-2404-wrapper",
                    "PLATFORM_PLUG=kde-qt6-core24",
                ],
            }
        }
    )
