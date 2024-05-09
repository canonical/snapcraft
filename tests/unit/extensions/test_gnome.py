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

from snapcraft.extensions import gnome
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def gnome_extension():
    return gnome.GNOME(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def gnome_extension_with_build_snap():
    return gnome.GNOME(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["gnome-44-2204-sdk"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def gnome_extension_with_default_build_snap_from_latest_edge():
    return gnome.GNOME(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["gnome-42-2204-sdk/latest/edge"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# GNOME Extension #
###################


def test_get_supported_bases():
    assert gnome.GNOME.get_supported_bases() == ("core22", "core24")


def test_get_supported_confinement():
    assert gnome.GNOME.get_supported_confinement() == ("strict", "devmode")


@pytest.mark.parametrize(
    ("base", "is_experimental"),
    [
        ("core22", False),
        ("core24", True),
    ],
)
def test_is_experimental(base, is_experimental):
    assert gnome.GNOME.is_experimental(base=base) is is_experimental


def test_get_app_snippet(gnome_extension):
    assert gnome_extension.get_app_snippet() == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "gsettings", "opengl", "wayland", "x11"],
    }


def test_get_root_snippet(gnome_extension):
    assert gnome_extension.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {
            "GTK_USE_PORTAL": "1",
            "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
        },
        "hooks": {
            "configure": {
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                "plugs": ["desktop"],
            }
        },
        "layout": {
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
            },
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            },
            "/usr/share/libdrm": {"bind": "$SNAP/gnome-platform/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gnome-42-2204": {
                "default-provider": "gnome-42-2204",
                "interface": "content",
                "target": "$SNAP/gnome-platform",
            },
            "gtk-3-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
            },
            "icon-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
            },
            "sound-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
            },
        },
    }


def test_get_root_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {
            "GTK_USE_PORTAL": "1",
            "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
        },
        "hooks": {
            "configure": {
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                "plugs": ["desktop"],
            }
        },
        "layout": {
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
            },
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            },
            "/usr/share/libdrm": {"bind": "$SNAP/gnome-platform/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gnome-44-2204": {
                "default-provider": "gnome-44-2204",
                "interface": "content",
                "target": "$SNAP/gnome-platform",
            },
            "gtk-3-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
            },
            "icon-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
            },
            "sound-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for GNOME.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, gnome_extension):
        self.assert_get_part_snippet(gnome_extension)

    def test_get_part_snippet_latest_edge(
        self, gnome_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            gnome_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(gnome_instance):
        assert gnome_instance.get_part_snippet(plugin_name="autotools") == {
            "build-environment": [
                {"PATH": "/snap/gnome-42-2204-sdk/current/usr/bin${PATH:+:$PATH}"},
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:/snap/gnome-42-2204-sdk"
                        "/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "LD_LIBRARY_PATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            "/snap/gnome-42-2204-sdk/current/usr/lib",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/vala-current",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio",
                        ]
                    )
                    + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                },
                {
                    "PKG_CONFIG_PATH": (
                        "/snap/gnome-42-2204-sdk/current/usr/lib/"
                        "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:"
                        "/snap/gnome-42-2204-sdk/current/usr/lib/pkgconfig:"
                        "/snap/gnome-42-2204-sdk/current/usr/share/pkgconfig"
                        "${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
                    )
                },
                {
                    "GETTEXTDATADIRS": (
                        "/snap/gnome-42-2204-sdk/current/usr/share/gettext-current"
                        "${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}"
                    )
                },
                {
                    "GDK_PIXBUF_MODULE_FILE": (
                        "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                        "/gdk-pixbuf-current/loaders.cache"
                    )
                },
                {
                    "ACLOCAL_PATH": (
                        "/snap/gnome-42-2204-sdk/current/usr/share/aclocal"
                        "${ACLOCAL_PATH:+:$ACLOCAL_PATH}"
                    )
                },
                {
                    "PYTHONPATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/usr/lib/python3.10",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/python3/dist-packages",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                            "/gobject-introspection",
                        ]
                    )
                    + "${PYTHONPATH:+:$PYTHONPATH}"
                },
                {
                    "GI_TYPELIB_PATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/usr/lib/girepository-1.0",
                            (
                                "/snap/gnome-42-2204-sdk/current/usr/lib/"
                                "$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0"
                            ),
                        ]
                    )
                    + "${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}"
                },
            ]
        }


def test_get_part_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_part_snippet(plugin_name="meson") == {
        "build-environment": [
            {"PATH": "/snap/gnome-44-2204-sdk/current/usr/bin${PATH:+:$PATH}"},
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:/snap/gnome-44-2204-sdk"
                    "/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "LD_LIBRARY_PATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                        "/snap/gnome-44-2204-sdk/current/usr/lib",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/vala-current",
                        (
                            "/snap/gnome-44-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio"
                        ),
                    ]
                )
                + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            },
            {
                "PKG_CONFIG_PATH": (
                    "/snap/gnome-44-2204-sdk/current/usr/lib/"
                    "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:"
                    "/snap/gnome-44-2204-sdk/current/usr/lib/pkgconfig:"
                    "/snap/gnome-44-2204-sdk/current/usr/share/pkgconfig"
                    "${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
                )
            },
            {
                "GETTEXTDATADIRS": (
                    "/snap/gnome-44-2204-sdk/current/usr/share/gettext-current"
                    "${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}"
                )
            },
            {
                "GDK_PIXBUF_MODULE_FILE": (
                    "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                    "/gdk-pixbuf-current/loaders.cache"
                )
            },
            {
                "ACLOCAL_PATH": (
                    "/snap/gnome-44-2204-sdk/current/usr/share/aclocal"
                    "${ACLOCAL_PATH:+:$ACLOCAL_PATH}"
                )
            },
            {
                "PYTHONPATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/usr/lib/python3.10",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/python3/dist-packages",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                        "/gobject-introspection",
                    ]
                )
                + "${PYTHONPATH:+:$PYTHONPATH}"
            },
            {
                "GI_TYPELIB_PATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/usr/lib/girepository-1.0",
                        (
                            "/snap/gnome-44-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0"
                        ),
                    ]
                )
                + "${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}"
            },
        ]
    }


def test_get_parts_snippet(gnome_extension):
    assert gnome_extension.get_parts_snippet() == {
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
            "build-snaps": ["gnome-42-2204-sdk"],
        }
    }


def test_get_parts_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_parts_snippet() == {
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    gnome_extension_with_default_build_snap_from_latest_edge,
):
    assert (
        gnome_extension_with_default_build_snap_from_latest_edge.get_parts_snippet()
        == {
            "gnome/sdk": {
                "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
                "plugin": "make",
            }
        }
    )
