# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd
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

from snapcraft.extensions import extension, gnome

############
# Fixtures #
############


@pytest.fixture
def gnome_extension():
    return gnome.GNOME(yaml_data={"base": "core22"}, arch="amd64", target_arch="amd64")


###################
# GNOME Extension #
###################


def test_get_supported_bases(gnome_extension):
    assert gnome_extension.get_supported_bases() == ("core22",)


def test_get_supported_confinement(gnome_extension):
    assert gnome_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental():
    assert gnome.GNOME.is_experimental(base="core22") is False


def test_get_app_snippet(gnome_extension):
    assert gnome_extension.get_app_snippet() == {
        "command-chain": ["$SNAP/gnome-platform/desktop-launch"],
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
            "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
            },
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            },
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


def test_get_part_snippet(gnome_extension):
    assert gnome_extension.get_part_snippet() == {
        "build-environment": [
            {"PATH": "$PATH:/snap/gnome-42-2204-sdk/current/usr/bin"},
            {
                "XDG_DATA_DIRS": (
                    "$XDG_DATA_DIRS:$SNAPCRAFT_STAGE/usr/share:/snap/gnome-42-2204-sdk"
                    "/current/usr/share:/usr/share"
                )
            },
            {
                "LD_LIBRARY_PATH": "${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}"
                + ":".join(
                    [
                        "/snap/gnome-42-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET",
                        "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET",
                        "/snap/gnome-42-2204-sdk/current/usr/lib",
                        "/snap/gnome-42-2204-sdk/current/usr/lib/vala-current",
                        "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio",
                    ]
                )
            },
            {
                "PKG_CONFIG_PATH": (
                    "$PKG_CONFIG_PATH:"
                    "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig:"
                    "/snap/gnome-42-2204-sdk/current/usr/lib/pkgconfig:"
                    "/snap/gnome-42-2204-sdk/current/usr/share/pkgconfig"
                )
            },
            {
                "GETTEXTDATADIRS": (
                    "$GETTEXTDATADIRS:"
                    "/snap/gnome-42-2204-sdk/current/usr/share/gettext-current"
                )
            },
            {
                "GDK_PIXBUF_MODULE_FILE": (
                    "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET"
                    "/gdk-pixbuf-current/loaders.cache"
                )
            },
            {
                "ACLOCAL_PATH": (
                    "${ACLOCAL_PATH:+$ACLOCAL_PATH:}"
                    "/snap/gnome-42-2204-sdk/current/usr/share/aclocal"
                )
            },
            {
                "PYTHONPATH": "${PYTHONPATH:+$PYTHONPATH:}"
                + ":".join(
                    [
                        "/snap/gnome-42-2204-sdk/current/usr/lib/python3.10",
                        "/snap/gnome-42-2204-sdk/current/usr/lib/python3/dist-packages",
                    ]
                )
            },
        ]
    }


def test_get_parts_snippet(gnome_extension):
    assert gnome_extension.get_parts_snippet() == {
        "gnome/hook-configure-fonts": {
            "source": str(extension.get_extensions_data_dir() / "fonts"),
            "plugin": "nil",
            "override-build": (
                "install -D -m 0755 hooks-configure-fonts "
                "${SNAPCRAFT_PART_INSTALL}/snap/command-chain/"
            ),
        }
    }
