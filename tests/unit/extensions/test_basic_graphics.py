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

from snapcraft.extensions import basic_graphics
from snapcraft.extensions.extension import get_extensions_data_dir

@pytest.fixture
def basic_graphics_extension():
    return basic_graphics.BasicGraphics(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


def test_get_supported_bases(basic_graphics_extension):
    assert basic_graphics_extension.get_supported_bases() == ("core22",)


def test_get_supported_confinement(basic_graphics_extension):
    assert basic_graphics_extension.get_supported_confinement() == ("strict", "devmode")


def test_is_experimental(basic_graphics_extension):
    assert basic_graphics_extension.is_experimental(base="core22") is False


def test_get_root_snippet(basic_graphics_extension):
    assert basic_graphics_extension.get_root_snippet() == {
        "plugs": {
            "graphics-core22": {
                "interface": "content",
                "target": "$SNAP/graphics",
                "default-provider": "mesa-core22",
            },
        },
        "layout": {
            "/usr/share/drirc.d": {
                "bind": "$SNAP/graphics/drirc.d",
            },
            "/usr/share/libdrm": {
                "bind": "$SNAP/graphics/libdrm",
            },
            "/usr/share/X11/XErrorDB": {
                "bind-file": "$SNAP/graphics/X11/XErrorDB",
            },
            "/usr/share/X11/locale": {
                "bind": "$SNAP/graphics/X11/locale",
            },
        },
    }


def test_get_app_snippet(basic_graphics_extension):
    assert basic_graphics_extension.get_app_snippet() == {
        "command-chain": [
            "bin/graphics-launch",
        ],
        "plugs": ["opengl", "wayland", "x11"],
    }


def test_get_part_snippet(basic_graphics_extension):
    assert basic_graphics_extension.get_part_snippet(plugin_name="nil") == {}


def test_get_parts_snippet(basic_graphics_extension):
    assert basic_graphics_extension.get_parts_snippet() == {
        "basic-graphics/basic-graphics": {
            "source": str(get_extensions_data_dir() / "basic-graphics"),
            "plugin": "dump",
            "override-build": "craftctl default\n"
                              "sed -i 's/%GRAPHICS%/graphics-core22/g' $CRAFT_PART_INSTALL/bin/graphics-launch\n"
                              "sed -i 's/%MESA%/mesa-core22/g' $CRAFT_PART_INSTALL/bin/graphics-launch",
            "override-prime": "craftctl default\n"
                              "graphics-cleanup mesa-core22",
            "prime": ["bin/graphics-launch"],
            "after": list(),
        }
    }


