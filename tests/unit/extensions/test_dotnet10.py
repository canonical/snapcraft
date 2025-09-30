# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

from snapcraft.extensions.dotnet10 import Dotnet10Extension
from snapcraft.extensions.extension import get_extensions_data_dir

_DOTNET_RUNTIME_PLUG_NAME = "dotnet10-runtime"
_DOTNET_RUNTIME_CONTENT_SNAP_NAME = "dotnet-runtime-100"
_VERSIONED_PLUGIN_NAME = "dotnet10"

############
# Fixtures #
############


@pytest.fixture
def dotnet_extension():
    return Dotnet10Extension(
        yaml_data={"name": "test", "base": "core24", "parts": {}},
        arch="amd64",
        target_arch="amd64",
    )


##################
# .NET Extension #
##################


def test_get_supported_bases(dotnet_extension):
    assert dotnet_extension.get_supported_bases() == ("core24",)


def test_get_supported_confinement(dotnet_extension):
    assert dotnet_extension.get_supported_confinement() == (
        "strict",
        "devmode",
    )


def test_is_experimental():
    assert Dotnet10Extension.is_experimental(base="core24") is True


def test_get_root_snippet(dotnet_extension):
    assert dotnet_extension.get_root_snippet() == {
        "plugs": {
            _DOTNET_RUNTIME_PLUG_NAME: {
                "interface": "content",
                "default-provider": _DOTNET_RUNTIME_CONTENT_SNAP_NAME,
                "content": _DOTNET_RUNTIME_CONTENT_SNAP_NAME,
                "target": f"$SNAP/opt/{_VERSIONED_PLUGIN_NAME}",
            }
        }
    }


def test_get_app_snippet(dotnet_extension):
    assert dotnet_extension.get_app_snippet(app_name="test") == {
        "command-chain": ["bin/command-chain/launcher.sh"],
        "environment": {
            "DOTNET_EXT_CONTENT_SNAP": _DOTNET_RUNTIME_CONTENT_SNAP_NAME,
            "DOTNET_EXT_SNAP_NAME": "test",
            "DOTNET_EXT_PLUG_NAME": _DOTNET_RUNTIME_PLUG_NAME,
            "DOTNET_ROOT": f"$SNAP/opt/{_VERSIONED_PLUGIN_NAME}/dotnet",
        },
        "plugs": [_DOTNET_RUNTIME_PLUG_NAME],
    }


def test_get_parts_snippet(dotnet_extension):
    assert dotnet_extension.get_parts_snippet() == {
        f"{_VERSIONED_PLUGIN_NAME}/launcher": {
            "plugin": "dump",
            "source": f"{get_extensions_data_dir()}/dotnet",
            "override-build": """
mkdir -p $CRAFT_PART_INSTALL/bin/command-chain
cp launcher.sh $CRAFT_PART_INSTALL/bin/command-chain
""",
            "stage": [
                "bin/command-chain/launcher.sh",
            ],
        },
        f"{_VERSIONED_PLUGIN_NAME}/prereqs": {
            "plugin": "nil",
            "stage-packages": [
                "libicu74",
                "libunwind8",
                "libssl3t64",
                "liblttng-ust1t64",
                "libbrotli1",
            ],
        },
    }
