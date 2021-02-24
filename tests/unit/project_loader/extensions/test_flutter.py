# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import textwrap

import pytest

from snapcraft.internal.project_loader._extensions.flutter_dev import (
    ExtensionImpl as FlutterDevExtension,
)
from snapcraft.internal.project_loader._extensions.flutter_master import (
    ExtensionImpl as FlutterMasterExtension,
)


@pytest.fixture(params=[FlutterMasterExtension, FlutterDevExtension])
def extension_class(request):
    return request.param


def test_extension(extension_class):
    flutter_extension = extension_class(
        extension_name="flutter", yaml_data=dict(base="core18")
    )

    assert flutter_extension.root_snippet == {
        "assumes": ["snapd2.43"],
        "plugs": {
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
            "gnome-3-28-1804": {
                "interface": "content",
                "target": "$SNAP/gnome-platform",
                "default-provider": "gnome-3-28-1804",
            },
        },
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform"},
        "layout": {
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            }
        },
    }

    assert flutter_extension.app_snippet == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "gsettings", "opengl", "wayland", "x11"],
    }

    assert flutter_extension.part_snippet == dict()

    # TODO Hack until we move extension name to each extension class.
    channel = "dev" if "dev" in flutter_extension.__repr__() else "master"
    assert flutter_extension.parts == {
        "gnome-3-28-extension": {
            "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
            "source-subdir": "gnome",
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=gnome-3-28-1804"],
            "build-packages": ["gcc", "libgtk-3-dev"],
        },
        "flutter-extension": {
            "plugin": "nil",
            "override-pull": textwrap.dedent(
                f"""\
                            flutter channel {channel}
                            flutter config --enable-linux-desktop
                            flutter upgrade
                            flutter doctor
                            """
            ),
            "build-snaps": ["flutter/latest/stable"],
        },
    }


def test_supported_bases(extension_class):
    assert extension_class.get_supported_bases() == ("core18",)


def test_supported_confinement(extension_class):
    extension_class.get_supported_confinement() == ("strict", "devmode")
