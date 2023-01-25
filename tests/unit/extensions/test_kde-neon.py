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

from snapcraft.extensions import kde_neon
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def kde-neon_extension():
    return kde_neon.KDENeon
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def kde-neon_extension_with_build_snap():
    return kde_neon.KDENeon(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["kde-frameworks-5-102-qt-5-15-8-core22-sd/latest/stable"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def kde-neon_extension_with_default_build_snap_from_latest_edge():
    return kde_neon.KDENeon(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["kde-frameworks-5-102-qt-5-15-8-core22-sd/latest/edge"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# KDENeon Extension #
###################


def test_get_supported_bases(kde-neon_extension):
    assert kde-neon_extension.get_supported_bases() == ("core22",)


def test_get_supported_confinement(kde-neon_extension):
    assert kde-neon_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert kde_neon.KDENeon.is_experimental(base="core22") is False


def test_get_app_snippet(kde-neon_extension):
    assert kde-neon_extension.get_app_snippet() == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop",
                "desktop-legacy",
                "opengl",
                "wayland",
                "x11"],
    }


def test_get_root_snippet(kde-neon_extension):
    assert kde-neon_extension.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
        "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"}},
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
                "kde-frameworks-5-102-qt-5-15-8-core22": {
                    "content": "kde-frameworks-5-102-qt-5-15-8-core22-all",
                    "interface": "content",
                    "default-provider": "kde-frameworks-5-102-qt-5-15-8-core22",
                    "target": "$SNAP/kf5",
                },
        },
    }


def test_get_root_snippet_with_external_sdk(kde-neon_extension_with_build_snap):
    assert kde-neon_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
        "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
        },
        "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"}},
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
                "kde-frameworks-5-102-qt-5-15-8-core22": {
                    "content": "kde-frameworks-5-102-qt-5-15-8-core22-all",
                    "interface": "content",
                    "default-provider": "kde-frameworks-5-102-qt-5-15-8-core22",
                    "target": "$SNAP/kf5",
                },
        },
    }


class TestGetPartSnippet:
    """Tests for KDENeon.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, kde-neon_extension):
        self.assert_get_part_snippet(kde-neon_extension)

    def test_get_part_snippet_latest_edge(
        self, kde-neon_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            kde-neon_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(kde-neon_instance):
        assert kde-neon_instance.get_part_snippet() == {
             "build-environment": [
                {
                    "PATH": prepend_to_env(
                        "PATH", [f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/bin"]
                    ),
                },
                {
                    "XDG_DATA_DIRS": prepend_to_env(
                        "XDG_DATA_DIRS",
                        [
                            f"$SNAPCRAFT_STAGE/usr/share:/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share",
                            "/usr/share",
                        ],
                    ),
                },
                {
                    "LD_LIBRARY_PATH": prepend_to_env(
                        "LD_LIBRARY_PATH",
                        [
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/lib/$CRAFT_ARCH_TRIPLET",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/vala-current",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio",
                        ],
                    ),
                },
                {
                    "PKG_CONFIG_PATH": prepend_to_env(
                        "PKG_CONFIG_PATH",
                        [
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/pkgconfig",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/pkgconfig",
                        ],
                    ),
                },
                {
                    "GETTEXTDATADIRS": prepend_to_env(
                        "GETTEXTDATADIRS",
                        [
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/gettext-current",
                        ],
                    ),
                },
                {
                    "ACLOCAL_PATH": prepend_to_env(
                        "ACLOCAL_PATH",
                        [
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/aclocal",
                        ],
                    ),
                },
                {
                    "PYTHONPATH": prepend_to_env(
                        "PYTHONPATH",
                        [
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/python3.10",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/python3/dist-packages",
                            f"/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET"
                            "/gobject-introspection",
                        ],
                    ),
                },
                {
                    "SNAPCRAFT_CMAKE_ARGS": prepend_to_env(
                        "SNAPCRAFT_CMAKE_ARGS",
                        ["-DCMAKE_FIND_ROOT_PATH=/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current"],
                    ),
                },
            ],
        }


def test_get_part_snippet_with_external_sdk(kde-neon_extension_with_build_snap):
    assert kde-neon_extension_with_build_snap.get_part_snippet() == {
        "build-environment": [
            {"PATH": "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/bin${PATH:+:$PATH}"},
            {
                "XDG_DATA_DIRS": (
                    "$SNAPCRAFT_STAGE/usr/share:/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd"
                    "/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "LD_LIBRARY_PATH": ":".join(
                    [
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/lib/$CRAFT_ARCH_TRIPLET",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/vala-current",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio",
                    ]
                )
                + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            },
            {
                "PKG_CONFIG_PATH": (
                    "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig:"
                    "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/pkgconfig:"
                    "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/pkgconfig"
                    "${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
                )
            },
            {
                "GETTEXTDATADIRS": (
                    "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/gettext-current"
                    "${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}"
                )
            },
            {
                "ACLOCAL_PATH": (
                    "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/share/aclocal"
                    "${ACLOCAL_PATH:+:$ACLOCAL_PATH}"
                )
            },
            {
                "PYTHONPATH": ":".join(
                    [
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/python3.10",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/python3/dist-packages",
                        "/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current/usr/lib/$CRAFT_ARCH_TRIPLET"
                        "/gobject-introspection",
                    ]
                )
                + "${PYTHONPATH:+:$PYTHONPATH}"
            },
            {
                "SNAPCRAFT_CMAKE_ARGS": (
                    "-DCMAKE_FIND_ROOT_PATH=/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current"
                )
            },
        ]
    }


def test_get_parts_snippet(kde-neon_extension):
    assert kde-neon_extension.get_parts_snippet() == {
        "kde-neon-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "kde-neon",
                "plugin": "make",
                "make-parameters": [f"PLATFORM_PLUG={"kde-frameworks-5-102-qt-5-15-8-core22-all"}"],
                "build-packages": ["g++"],
                "build-snaps": ["kde-frameworks-5-102-qt-5-15-8-core22-sd"],
            }
    }

def test_get_parts_snippet_with_external_sdk(kde-neon_extension_with_build_snap):
    assert kde-neon_extension_with_build_snap.get_parts_snippet() == {
        "kde-neon-extension": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    kde-neon_extension_with_default_build_snap_from_latest_edge,
):
    assert (
        kde-neon_extension_with_default_build_snap_from_latest_edge.get_parts_snippet()
        == {
            "kde-neon-extension": {
                "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
                "plugin": "make",
            }
        }
    )
