# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from snapcraft.internal.project_loader._extensions.kde_neon import ExtensionImpl


def test_extension_core18():
    kde_neon_extension = ExtensionImpl(
        extension_name="kde-neon", yaml_data=dict(base="core18")
    )

    assert kde_neon_extension.root_snippet == {
        "assumes": ["snapd2.43"],
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
            "kde-frameworks-5-plug": {
                "content": "kde-frameworks-5-core18-all",
                "interface": "content",
                "target": "$SNAP/kf5",
                "default-provider": "kde-frameworks-5-core18",
            },
        },
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
        "hooks": {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        },
    }
    assert kde_neon_extension.app_snippet == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
    }
    assert kde_neon_extension.part_snippet == dict()
    assert kde_neon_extension.parts == {
        "kde-neon-extension": {
            "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
            "source-subdir": "kde-neon",
            "plugin": "make",
            "make-parameters": ["PLATFORM_PLUG=kde-frameworks-5-plug"],
            "build-packages": ["g++"],
            "build-snaps": ["kde-frameworks-5-core18-sdk/latest/stable"],
        }
    }


def test_extension_core20():
    kde_neon_extension = ExtensionImpl(
        extension_name="kde-neon", yaml_data=dict(base="core20")
    )

    assert kde_neon_extension.root_snippet == {
        "assumes": ["snapd2.43"],
        "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
        "hooks": {
            "configure": {
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                "plugs": ["desktop"],
            }
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "icon-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
            },
            "kde-frameworks-5-plug": {
                "content": "kde-frameworks-5-qt-5-15-core20-all",
                "default-provider": "kde-frameworks-5-qt-5-15-core20",
                "interface": "content",
                "target": "$SNAP/kf5",
            },
            "sound-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
            },
        },
    }
    assert kde_neon_extension.app_snippet == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
    }
    assert kde_neon_extension.part_snippet == {
        "build-environment": [
            {
                "SNAPCRAFT_CMAKE_ARGS": "-DCMAKE_FIND_ROOT_PATH=/snap/kde-frameworks-5-qt-5-15-core20-sdk/current"
            }
        ]
    }
    assert kde_neon_extension.parts == {
        "kde-neon-extension": {
            "build-packages": ["g++"],
            "build-snaps": ["kde-frameworks-5-qt-5-15-core20-sdk/latest/candidate"],
            "make-parameters": ["PLATFORM_PLUG=kde-frameworks-5-plug"],
            "plugin": "make",
            "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
            "source-subdir": "kde-neon",
        }
    }


def test_supported_bases():
    assert ExtensionImpl.get_supported_bases() == ("core18", "core20")


def test_supported_confinement():
    assert ExtensionImpl.get_supported_confinement() == ("strict", "devmode")


def test_experimental_core20():
    kde_neon_extension = ExtensionImpl(
        extension_name="kde-neon", yaml_data=dict(base="core20")
    )
    assert kde_neon_extension.is_experimental(base="core20") is True


def test_experimental_core18():
    kde_neon_extension = ExtensionImpl(
        extension_name="kde-neon", yaml_data=dict(base="core18")
    )
    assert kde_neon_extension.is_experimental(base="core18") is False
