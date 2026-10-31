# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
# Copyright 2023-2024 Scarlett Moore <sgmoore@kde.org>
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
    return kde_neon_6.KDENeon6(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_6_extension_core24():
    return kde_neon_6.KDENeon6(
        yaml_data={"base": "core24", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_6_extension_with_build_snap():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core22-sdk/latest/stable",
                        "kf6-core22-sdk/latest/stable",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_6_extension_with_build_snap_core24():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core24",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core24-sdk/latest/stable",
                        "kf6-core24-sdk/latest/stable",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_6_extension_with_default_build_snap_from_latest_edge():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core22",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core22-sdk/latest/edge",
                        "kf6-core22-sdk/latest/edge",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde_neon_6_extension_with_default_build_snap_from_latest_edge_core24():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core24",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core24-sdk/latest/edge",
                        "kf6-core24-sdk/latest/edge",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# KDENeon6 Extension #
###################


def test_get_supported_bases(kde_neon_6_extension):
    assert kde_neon_6_extension.get_supported_bases() == ("core22", "core24", "core26")


def test_get_supported_confinement(kde_neon_6_extension):
    assert kde_neon_6_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert kde_neon_6.KDENeon6.is_experimental(base="core22") is False


def test_get_app_snippet(kde_neon_6_extension):
    assert kde_neon_6_extension.get_app_snippet(app_name="test-app") == {
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


def test_get_app_snippet_core24(kde_neon_6_extension_core24):
    assert kde_neon_6_extension_core24.get_app_snippet(app_name="test-app") == {
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


def test_get_root_snippet(kde_neon_6_extension):
    assert kde_neon_6_extension.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kf6-core22",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"bind": "$SNAP/kf6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/kf6-core22/usr/share/libdrm"},
            "/usr/share/Kvantum": {"symlink": "$SNAP/lxqt-support/usr/share/Kvantum"},
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
            "kf6-core22": {
                "content": "kf6-core22-all",
                "interface": "content",
                "default-provider": "kf6-core22",
                "target": "$SNAP/kf6",
            },
            "lxqt-support-core22": {
                "content": "lxqt-support-core22",
                "interface": "content",
                "default-provider": "lxqt-support-core22",
                "target": "$SNAP/lxqt-support",
            },
        },
    }


def test_get_root_snippet_core24(kde_neon_6_extension_core24):
    assert kde_neon_6_extension_core24.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kf6-core24",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"bind": "$SNAP/kf6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            "/usr/share/Kvantum": {"symlink": "$SNAP/lxqt-support/usr/share/Kvantum"},
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
            "kf6-core24": {
                "content": "kf6-core24-all",
                "interface": "content",
                "default-provider": "kf6-core24",
                "target": "$SNAP/kf6",
            },
            "gpu-2404": {
                "default-provider": "mesa-2404",
                "interface": "content",
                "target": "$SNAP/gpu-2404",
            },
            "lxqt-support-core24": {
                "content": "lxqt-support-core24",
                "interface": "content",
                "default-provider": "lxqt-support-core24",
                "target": "$SNAP/lxqt-support",
            },
        },
    }


def test_get_root_snippet_with_gpu(kde_neon_6_extension_core24):
    snippet = kde_neon_6_extension_core24.get_root_snippet()
    assert snippet["plugs"]["gpu-2404"] == {
        "default-provider": "mesa-2404",
        "interface": "content",
        "target": "$SNAP/gpu-2404",
    }


def test_get_root_snippet_without_gpu(kde_neon_6_extension):
    snippet = kde_neon_6_extension.get_root_snippet()
    assert snippet["layout"]["/usr/share/libdrm"] == {
        "bind": "$SNAP/kf6-core22/usr/share/libdrm",
    }


def test_get_root_snippet_no_exceptions(kde_neon_6_extension):
    with pytest.raises(AssertionError):
        assert not kde_neon_6_extension.get_root_snippet()


def test_get_root_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    assert kde_neon_6_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kf6-core22",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"bind": "$SNAP/kf6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            "/usr/share/libdrm": {"bind": "$SNAP/kf6-core22/usr/share/libdrm"},
            "/usr/share/Kvantum": {"symlink": "$SNAP/lxqt-support/usr/share/Kvantum"},
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
            "kf6-core22": {
                "content": "kf6-core22-all",
                "interface": "content",
                "default-provider": "kf6-core22",
                "target": "$SNAP/kf6",
            },
            "lxqt-support-core22": {
                "content": "lxqt-support-core22",
                "interface": "content",
                "default-provider": "lxqt-support-core22",
                "target": "$SNAP/lxqt-support",
            },
        },
    }


def test_get_root_snippet_with_external_sdk_core24(
    kde_neon_6_extension_with_build_snap_core24,
):
    assert kde_neon_6_extension_with_build_snap_core24.get_root_snippet() == {
        "assumes": ["snapd2.58.3"],
        "compression": "lzo",
        "environment": {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": "kf6-core24",
        },
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
        "layout": {
            "/usr/share/X11": {"bind": "$SNAP/kf6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            "/usr/share/Kvantum": {"symlink": "$SNAP/lxqt-support/usr/share/Kvantum"},
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
            "kf6-core24": {
                "content": "kf6-core24-all",
                "interface": "content",
                "default-provider": "kf6-core24",
                "target": "$SNAP/kf6",
            },
            "gpu-2404": {
                "default-provider": "mesa-2404",
                "interface": "content",
                "target": "$SNAP/gpu-2404",
            },
            "lxqt-support-core24": {
                "content": "lxqt-support-core24",
                "interface": "content",
                "default-provider": "lxqt-support-core24",
                "target": "$SNAP/lxqt-support",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for KDENeon6.get_part_snippet when using the default sdk snap name."""

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
                        "$CRAFT_STAGE/usr/bin:"
                        "/snap/kde-qt6-core22-sdk/current/usr/bin:"
                        "/snap/kf6-core22-sdk/current/usr/bin"
                        "${PATH:+:$PATH}"
                    )
                },
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:"
                        "/snap/kde-qt6-core22-sdk/current/usr/share:"
                        "/snap/kf6-core22-sdk/current/usr/share"
                        "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "XDG_CONFIG_HOME": (
                        "$CRAFT_STAGE/etc/xdg:"
                        "/snap/kde-qt6-core22-sdk/current/etc/xdg:"
                        "/snap/kf6-core22-sdk/current/etc/xdg"
                        "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                    )
                },
                {
                    "LD_LIBRARY_PATH": (
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib:"
                        "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/blas:"
                        "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/lapack:"
                        "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/libproxy:"
                        "/snap/kf6-core22-sdk/current/usr/lib:"
                        "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "$CRAFT_STAGE/usr/lib:"
                        "$CRAFT_STAGE/lib/"
                        "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                    )
                },
                {
                    "CMAKE_PREFIX_PATH": (
                        "$CRAFT_STAGE/usr:"
                        "/snap/kde-qt6-core22-sdk/current/usr:"
                        "/snap/kf6-core22-sdk/current/usr:"
                        "/usr"
                        "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                    )
                },
            ]
        }

    def test_get_part_snippet_core24(self, kde_neon_6_extension_core24):
        self.assert_get_part_snippet_core24(kde_neon_6_extension_core24)

    def test_get_part_snippet_latest_edge_core24(
        self, kde_neon_6_extension_with_default_build_snap_from_latest_edge_core24
    ):
        self.assert_get_part_snippet_core24(
            kde_neon_6_extension_with_default_build_snap_from_latest_edge_core24
        )

    @staticmethod
    def assert_get_part_snippet_core24(kde_neon_instance):
        assert kde_neon_instance.get_part_snippet(plugin_name="cmake") == {
            "build-environment": [
                {
                    "PATH": (
                        "$CRAFT_STAGE/usr/bin:"
                        "/snap/kde-qt6-core24-sdk/current/usr/bin:"
                        "/snap/kf6-core24-sdk/current/usr/bin"
                        "${PATH:+:$PATH}"
                    )
                },
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:"
                        "/snap/kde-qt6-core24-sdk/current/usr/share:"
                        "/snap/kf6-core24-sdk/current/usr/share"
                        "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "XDG_CONFIG_HOME": (
                        "$CRAFT_STAGE/etc/xdg:"
                        "/snap/kde-qt6-core24-sdk/current/etc/xdg:"
                        "/snap/kf6-core24-sdk/current/etc/xdg"
                        "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                    )
                },
                {
                    "LD_LIBRARY_PATH": (
                        "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/kde-qt6-core24-sdk/current/usr/lib:"
                        "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/mesa-2404/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/blas:"
                        "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/lapack:"
                        "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                        "/libproxy:"
                        "/snap/kf6-core24-sdk/current/usr/lib:"
                        "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                        "$CRAFT_STAGE/usr/lib:"
                        "$CRAFT_STAGE/lib/"
                        "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                    )
                },
                {
                    "CMAKE_PREFIX_PATH": (
                        "$CRAFT_STAGE/usr:"
                        "/snap/kde-qt6-core24-sdk/current/usr:"
                        "/snap/kf6-core24-sdk/current/usr:"
                        "/usr"
                        "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                    )
                },
            ]
        }


def test_get_part_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    assert kde_neon_6_extension_with_build_snap.get_part_snippet(
        plugin_name="cmake"
    ) == {
        "build-environment": [
            {
                "PATH": (
                    "$CRAFT_STAGE/usr/bin:"
                    "/snap/kde-qt6-core22-sdk/current/usr/bin:"
                    "/snap/kf6-core22-sdk/current/usr/bin"
                    "${PATH:+:$PATH}"
                )
            },
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:"
                    "/snap/kde-qt6-core22-sdk/current/usr/share:"
                    "/snap/kf6-core22-sdk/current/usr/share"
                    "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "XDG_CONFIG_HOME": (
                    "$CRAFT_STAGE/etc/xdg:"
                    "/snap/kde-qt6-core22-sdk/current/etc/xdg:"
                    "/snap/kf6-core22-sdk/current/etc/xdg"
                    "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                )
            },
            {
                "LD_LIBRARY_PATH": (
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib:"
                    "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas:"
                    "/snap/kf6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack:"
                    "/snap/kde-qt6-core22-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libproxy:"
                    "/snap/kf6-core22-sdk/current/usr/lib:"
                    "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "$CRAFT_STAGE/usr/lib:"
                    "$CRAFT_STAGE/lib/"
                    "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                ),
            },
            {
                "CMAKE_PREFIX_PATH": (
                    "$CRAFT_STAGE/usr:"
                    "/snap/kde-qt6-core22-sdk/current/usr:"
                    "/snap/kf6-core22-sdk/current/usr:"
                    "/usr"
                    "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                )
            },
        ]
    }


def test_get_part_snippet_with_external_sdk_core24(
    kde_neon_6_extension_with_build_snap_core24,
):
    assert kde_neon_6_extension_with_build_snap_core24.get_part_snippet(
        plugin_name="cmake"
    ) == {
        "build-environment": [
            {
                "PATH": (
                    "$CRAFT_STAGE/usr/bin:"
                    "/snap/kde-qt6-core24-sdk/current/usr/bin:"
                    "/snap/kf6-core24-sdk/current/usr/bin"
                    "${PATH:+:$PATH}"
                )
            },
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:"
                    "/snap/kde-qt6-core24-sdk/current/usr/share:"
                    "/snap/kf6-core24-sdk/current/usr/share"
                    "${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "XDG_CONFIG_HOME": (
                    "$CRAFT_STAGE/etc/xdg:"
                    "/snap/kde-qt6-core24-sdk/current/etc/xdg:"
                    "/snap/kf6-core24-sdk/current/etc/xdg"
                    "${XDG_CONFIG_HOME:+:$XDG_CONFIG_HOME}"
                )
            },
            {
                "LD_LIBRARY_PATH": (
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib:"
                    "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/mesa-2404/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/blas:"
                    "/snap/kf6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/lapack:"
                    "/snap/kde-qt6-core24-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                    "/libproxy:"
                    "/snap/kf6-core24-sdk/current/usr/lib:"
                    "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}:"
                    "$CRAFT_STAGE/usr/lib:"
                    "$CRAFT_STAGE/lib/"
                    "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                )
            },
            {
                "CMAKE_PREFIX_PATH": (
                    "$CRAFT_STAGE/usr:"
                    "/snap/kde-qt6-core24-sdk/current/usr:"
                    "/snap/kf6-core24-sdk/current/usr:"
                    "/usr"
                    "${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
                )
            },
        ]
    }


def test_get_parts_snippet(kde_neon_6_extension):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_6_extension.get_parts_snippet() == {
        "kde-neon-6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "PLATFORM_PLUG=kf6-core22",
            ],
            "build-snaps": ["kde-qt6-core22-sdk", "kf6-core22-sdk"],
            "build-packages": [
                "gettext",
                "doxygen",
                "graphviz",
                "libxml2-utils",
                "docbook-xml",
                "docbook-xsl",
            ],
        }
    }


def test_get_parts_snippet_core24(kde_neon_6_extension_core24):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_6_extension_core24.get_parts_snippet() == {
        "kde-neon-6/gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=gpu-2404"],
        },
        "kde-neon-6/gpu/cleanup": {
            "after": [],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n"
                "${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-2404\n"
                "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                'mkdir -p "${CRAFT_PRIME}/gpu-2404"'
            ),
        },
        "kde-neon-6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "PLATFORM_PLUG=kf6-core24",
            ],
            "build-snaps": ["kde-qt6-core24-sdk", "kf6-core24-sdk"],
            "build-packages": [
                "gettext",
                "doxygen",
                "graphviz",
                "libxml2-utils",
                "docbook-xml",
                "docbook-xsl",
            ],
        },
    }


def test_get_parts_snippet_with_external_sdk(kde_neon_6_extension_with_build_snap):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

    assert kde_neon_6_extension_with_build_snap.get_parts_snippet() == {
        "kde-neon-6/sdk": {
            "source": str(source),
            "plugin": "make",
            "make-parameters": [
                "PLATFORM_PLUG=kf6-core22",
            ],
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    kde_neon_6_extension_with_default_build_snap_from_latest_edge_core24,
):
    source = get_extensions_data_dir() / "desktop" / "command-chain-kde"
    assert (
        kde_neon_6_extension_with_default_build_snap_from_latest_edge_core24.get_parts_snippet()
        == {
            "kde-neon-6/gpu/wrapper": {
                "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
                "plugin": "make",
                "make-parameters": ["GPU_INTERFACE=gpu-2404"],
            },
            "kde-neon-6/gpu/cleanup": {
                "after": ["part1"],
                "source": "https://github.com/canonical/gpu-snap.git",
                "plugin": "nil",
                "override-prime": (
                    "craftctl default\n"
                    "${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-2404\n"
                    "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                    'mkdir -p "${CRAFT_PRIME}/gpu-2404"'
                ),
            },
            "kde-neon-6/sdk": {
                "source": str(source),
                "plugin": "make",
                "make-parameters": [
                    "PLATFORM_PLUG=kf6-core24",
                ],
            },
        }
    )


@pytest.fixture
def kde_neon_6_extension_core26():
    return kde_neon_6.KDENeon6(
        yaml_data={"base": "core26", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde_neon_6_extension_with_build_snap_core26():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "part1": {
                    "build-snaps": [
                        "kde-qt6-core26-sdk/latest/stable",
                        "kf6-core26-sdk/latest/stable",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


def test_get_supported_bases_includes_preliminary_core26(kde_neon_6_extension):
    assert kde_neon_6_extension.get_supported_bases() == ("core22", "core24", "core26")


def test_is_experimental_core26():
    assert kde_neon_6.KDENeon6.is_experimental(base="core26") is True


def test_is_experimental_core24():
    assert kde_neon_6.KDENeon6.is_experimental(base="core24") is False


def test_core22_and_core24_are_not_experimental():
    assert kde_neon_6.KDENeon6.is_experimental(base="core22") is False
    assert kde_neon_6.KDENeon6.is_experimental(base="core24") is False


def test_core26_default_sdk_names(kde_neon_6_extension_core26):
    snaps = kde_neon_6_extension_core26.kde_snaps
    assert snaps.qt6_sdk_snap == "kde-qt6-core26-sdk"
    assert snaps.kf6_sdk_snap == "kf6-core26-sdk"
    assert snaps.content_qt6 == "kde-qt6-core26"
    assert snaps.content_kf6 == "kf6-core26"
    assert snaps.qt6_builtin is True
    assert snaps.kf6_builtin is True


def test_core26_external_sdk_names(kde_neon_6_extension_with_build_snap_core26):
    snaps = kde_neon_6_extension_with_build_snap_core26.kde_snaps
    assert snaps.qt6_sdk_snap == "kde-qt6-core26-sdk"
    assert snaps.kf6_sdk_snap == "kf6-core26-sdk"
    assert snaps.qt6_builtin is False
    assert snaps.kf6_builtin is False


def test_get_app_snippet_core26(kde_neon_6_extension_core26):
    snippet = kde_neon_6_extension_core26.get_app_snippet(app_name="test-app")
    assert snippet["command-chain"][-1] == "snap/command-chain/desktop-launch"
    assert "snap/command-chain/gpu-2604-wrapper" in snippet["command-chain"]
    assert snippet["plugs"] == [
        "desktop",
        "desktop-legacy",
        "opengl",
        "wayland",
        "x11",
        "audio-playback",
        "unity7",
        "network",
        "network-bind",
    ]


def test_get_root_snippet_core26(kde_neon_6_extension_core26):
    snippet = kde_neon_6_extension_core26.get_root_snippet()
    assert snippet["environment"] == {
        "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
        "GTK_USE_PORTAL": "1",
        "PLATFORM_PLUG": "kf6-core26",
    }
    assert snippet["plugs"]["kf6-core26"] == {
        "content": "kf6-core26-all",
        "interface": "content",
        "default-provider": "kf6-core26",
        "target": "$SNAP/kf6",
    }
    assert snippet["plugs"]["lxqt-support-core26"] == {
        "content": "lxqt-support-core26",
        "interface": "content",
        "target": "$SNAP/lxqt-support",
        "default-provider": "lxqt-support-core26",
    }
    assert snippet["layout"]["/usr/share/X11"] == {"bind": "$SNAP/kf6/usr/share/X11"}
    assert snippet["layout"]["/usr/share/qt6"] == {"symlink": "$SNAP/kf6/usr/share/qt6"}
    assert snippet["plugs"]["gpu-2604"] == {
        "default-provider": "mesa-2604",
        "interface": "content",
        "target": "$SNAP/gpu-2604",
    }


def test_get_part_snippet_core26(kde_neon_6_extension_core26):
    snippet = kde_neon_6_extension_core26.get_part_snippet(plugin_name="cmake")
    environment = snippet["build-environment"]
    assert "/snap/kde-qt6-core26-sdk/current/usr/bin" in environment[0]["PATH"]
    assert "/snap/kf6-core26-sdk/current/usr/bin" in environment[0]["PATH"]
    assert (
        "/snap/kde-qt6-core26-sdk/current/usr/share" in environment[1]["XDG_DATA_DIRS"]
    )
    assert "/snap/kf6-core26-sdk/current/etc/xdg" in environment[2]["XDG_CONFIG_HOME"]
    assert (
        "/snap/kde-qt6-core26-sdk/current/usr/lib" in environment[3]["LD_LIBRARY_PATH"]
    )
    assert "/snap/kf6-core26-sdk/current/usr/lib" in environment[3]["LD_LIBRARY_PATH"]
    assert (
        "/snap/mesa-2604/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
        in environment[3]["LD_LIBRARY_PATH"]
    )
    assert "mesa-2404" not in environment[3]["LD_LIBRARY_PATH"]
    cmake_prefix_path = next(
        item["CMAKE_PREFIX_PATH"] for item in environment if "CMAKE_PREFIX_PATH" in item
    )
    assert "/snap/kde-qt6-core26-sdk/current/usr" in cmake_prefix_path


def test_get_parts_snippet_core26(kde_neon_6_extension_core26):
    parts = kde_neon_6_extension_core26.get_parts_snippet()
    sdk_part = parts["kde-neon-6/sdk"]
    assert sdk_part["make-parameters"] == ["PLATFORM_PLUG=kf6-core26"]
    assert sdk_part["build-snaps"] == [
        "kde-qt6-core26-sdk",
        "kf6-core26-sdk",
        "kf6-core26",
    ]
    assert parts["kde-neon-6/gpu/wrapper"]["make-parameters"] == [
        "GPU_INTERFACE=gpu-2604"
    ]
    cleanup = parts["kde-neon-6/gpu/cleanup"]
    assert cleanup["source"] == "https://github.com/canonical/gpu-snap.git"
    assert "gpu-2604-cleanup mesa-2604" in cleanup["override-prime"]
    assert (
        "${CRAFT_PART_SRC}/bin/gpu-2604-cleanup mesa-2604"
        in (cleanup["override-prime"])
    )


def test_get_parts_snippet_with_external_sdk_core26(
    kde_neon_6_extension_with_build_snap_core26,
):
    parts = kde_neon_6_extension_with_build_snap_core26.get_parts_snippet()
    assert parts["kde-neon-6/sdk"]["make-parameters"] == ["PLATFORM_PLUG=kf6-core26"]
    assert parts["kde-neon-6/sdk"]["build-snaps"] == ["kf6-core26/latest/stable"]
    assert "build-packages" not in parts["kde-neon-6/sdk"]


@pytest.mark.parametrize("channel", ["latest/stable", "latest/beta", "latest/edge"])
def test_core26_runtime_channel_follows_kf6_sdk_channel(channel):
    extension = kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "app": {
                    "build-snaps": [
                        f"kde-qt6-core26-sdk/{channel}",
                        f"kf6-core26-sdk/{channel}",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )

    sdk_part = extension.get_parts_snippet()["kde-neon-6/sdk"]
    assert f"kf6-core26/{channel}" in sdk_part["build-snaps"]


def test_core26_runtime_channel_can_be_overridden_independently():
    extension = kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "app": {
                    "build-snaps": [
                        "kde-qt6-core26-sdk/latest/beta",
                        "kf6-core26-sdk/latest/beta",
                        "kf6-core26/latest/edge",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )

    sdk_part = extension.get_parts_snippet()["kde-neon-6/sdk"]
    assert "build-snaps" not in sdk_part


@pytest.fixture
def kde_neon_6_extension_with_pyside_core26():
    return kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "app": {
                    "build-snaps": [
                        "kde-qt6-core26-sdk/latest/beta",
                        "kf6-core26-sdk/latest/beta",
                        "kde-pyside6-core26-sdk/latest/beta",
                    ]
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


def test_pyside_sdk_is_opt_in_for_core26(kde_neon_6_extension_core26):
    assert kde_neon_6_extension_core26.pyside_sdk_snap is None
    environment = kde_neon_6_extension_core26.get_part_snippet(plugin_name="cmake")[
        "build-environment"
    ]
    assert not any("PYTHONPATH" in item for item in environment)


def test_core26_pyside_build_environment(
    kde_neon_6_extension_with_pyside_core26,
):
    extension = kde_neon_6_extension_with_pyside_core26
    assert extension.pyside_sdk_snap == "kde-pyside6-core26-sdk"

    environment = extension.get_part_snippet(plugin_name="cmake")["build-environment"]
    by_name = {key: value for item in environment for key, value in item.items()}

    assert "/snap/kde-pyside6-core26-sdk/current/usr/bin" in by_name["PATH"]
    assert (
        "/snap/kde-pyside6-core26-sdk/current/usr/lib/python3/dist-packages"
        in by_name["PYTHONPATH"]
    )
    assert "/snap/kde-pyside6-core26-sdk/current/usr/lib" in by_name["LD_LIBRARY_PATH"]
    assert (
        "/snap/kde-pyside6-core26-sdk/current/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
        in by_name["LD_LIBRARY_PATH"]
    )
    assert "/snap/kde-pyside6-core26-sdk/current/usr" in by_name["CMAKE_PREFIX_PATH"]
    # The KDE Qt SDK must precede the PySide SDK's library paths so Qt itself
    # resolves from the shared KDE SDK rather than a bundled duplicate.
    assert by_name["LD_LIBRARY_PATH"].index(
        "/snap/kde-qt6-core26-sdk/current/usr/lib"
    ) < by_name["LD_LIBRARY_PATH"].index(
        "/snap/kde-pyside6-core26-sdk/current/usr/lib:"
    )


def test_core22_does_not_add_pyside_environment(kde_neon_6_extension):
    environment = kde_neon_6_extension.get_part_snippet(plugin_name="cmake")[
        "build-environment"
    ]
    assert not any("PYTHONPATH" in item for item in environment)
    assert not any("kde-pyside6" in str(item) for item in environment)


def test_get_parts_snippet_with_qt_sdk_only_core26():
    extension = kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "app": {
                    "build-snaps": ["kde-qt6-core26-sdk/latest/stable"],
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )

    parts = extension.get_parts_snippet()
    sdk_part = parts["kde-neon-6/sdk"]

    assert sdk_part["build-snaps"] == [
        "kf6-core26-sdk",
    ]
    assert "build-packages" in sdk_part


def test_get_parts_snippet_with_kf6_sdk_only_core26():
    extension = kde_neon_6.KDENeon6(
        yaml_data={
            "base": "core26",
            "parts": {
                "app": {
                    "build-snaps": ["kf6-core26-sdk/latest/stable"],
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )

    parts = extension.get_parts_snippet()
    sdk_part = parts["kde-neon-6/sdk"]

    assert sdk_part["build-snaps"] == [
        "kde-qt6-core26-sdk",
    ]
    assert "build-packages" in sdk_part


def test_get_part_snippet_core26_adds_pulseaudio_linker_paths(
    kde_neon_6_extension_core26,
):
    snippet = kde_neon_6_extension_core26.get_part_snippet(plugin_name="cmake")
    build_environment = snippet["build-environment"]

    ldflags = next(item["LDFLAGS"] for item in build_environment if "LDFLAGS" in item)

    assert (
        "-Wl,-rpath-link,"
        "/snap/kde-qt6-core26-sdk/current/usr/lib/"
        "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/pulseaudio"
    ) in ldflags

    assert (
        "-Wl,-rpath-link,"
        "/snap/kf6-core26-sdk/current/usr/lib/"
        "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/pulseaudio"
    ) in ldflags
