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

from snapcraft.extensions import dotnet
from snapcraft.extensions.extension import get_extensions_data_dir

_DOTNET_RUNTIME_PLUG_NAME = "dotnet-runtime"

############
# Fixtures #
############


@pytest.fixture
def dotnet_extension():
    return dotnet.DotnetExtension(
        yaml_data={"name": "test", "base": "core24", "parts": {}},
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def dotnet_extension_fxdependent():
    return dotnet.DotnetExtension(
        yaml_data={
            "name": "test",
            "base": "core24",
            "parts": {
                "app": {
                    "plugin": "dotnet",
                    "source": "https://example.com/app",
                    "dotnet-version": "8.0",
                    "dotnet-self-contained": False,
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def dotnet_extension_self_contained():
    return dotnet.DotnetExtension(
        yaml_data={
            "name": "test",
            "base": "core24",
            "parts": {
                "app": {
                    "plugin": "dotnet",
                    "source": "https://example.com/app",
                    "dotnet-version": "8.0",
                    "dotnet-self-contained": True,
                }
            },
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def dotnet_extension_factory():
    def _factory(
        dotnet_version: str, dotnet_self_contained: bool
    ) -> dotnet.DotnetExtension:
        """Factory to create a DotnetExtension with specific parameters."""
        return dotnet.DotnetExtension(
            yaml_data={
                "name": "test",
                "base": "core24",
                "parts": {
                    "app": {
                        "plugin": "dotnet",
                        "source": "https://example.com/app",
                        "dotnet-version": dotnet_version,
                        "dotnet-self-contained": dotnet_self_contained,
                    }
                },
            },
            arch="amd64",
            target_arch="amd64",
        )

    return _factory


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
    assert dotnet.DotnetExtension.is_experimental(base="core24") is True


def test_get_root_snippet(dotnet_extension):
    assert dotnet_extension.get_root_snippet() == {}


def test_get_root_snippet_fxdependent(dotnet_extension_fxdependent):
    assert dotnet_extension_fxdependent.get_root_snippet() == {
        "plugs": {
            _DOTNET_RUNTIME_PLUG_NAME: {
                "interface": "content",
                "default-provider": dotnet_extension_fxdependent.get_application_details.content_snap,
                "content": dotnet_extension_fxdependent.get_application_details.content_snap,
                "target": "$SNAP/usr/lib",
            }
        }
    }


def test_get_root_snippet_self_contained(dotnet_extension_self_contained):
    assert dotnet_extension_self_contained.get_root_snippet() == {}


def test_get_app_snippet(dotnet_extension):
    assert dotnet_extension.get_app_snippet(app_name="test") == {}


def test_get_app_snippet_fxdependent(dotnet_extension_fxdependent):
    assert dotnet_extension_fxdependent.get_app_snippet(app_name="test") == {
        "command-chain": ["bin/command-chain/launcher.sh"],
        "environment": {
            "DOTNET_EXT_CONTENT_SNAP": "dotnet-runtime-80",
            "DOTNET_EXT_SNAP_NAME": "test",
            "DOTNET_EXT_PLUG_NAME": _DOTNET_RUNTIME_PLUG_NAME,
            "DOTNET_ROOT": "$SNAP/usr/lib/dotnet",
        },
        "plugs": [_DOTNET_RUNTIME_PLUG_NAME],
    }


def test_get_app_snippet_self_contained(dotnet_extension_self_contained):
    assert dotnet_extension_self_contained.get_app_snippet(app_name="test") == {}


def test_get_part_snippet_fxdependent(dotnet_extension_fxdependent):
    assert dotnet_extension_fxdependent.get_part_snippet(plugin_name="dotnet") == {}


def test_get_part_snippet_self_contained(dotnet_extension_self_contained):
    assert dotnet_extension_self_contained.get_part_snippet(plugin_name="dotnet") == {}


def test_get_parts_snippet(dotnet_extension):
    assert dotnet_extension.get_parts_snippet() == {
        "dotnet/prereqs": {
            "plugin": "nil",
            "stage-packages": [
                "libicu74",
            ],
        }
    }


def test_get_parts_snippet_fxdependent(dotnet_extension_fxdependent):
    assert dotnet_extension_fxdependent.get_parts_snippet() == {
        "dotnet/launcher": {
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
        "dotnet/prereqs": {
            "plugin": "nil",
            "stage-packages": [
                "libicu74",
            ],
        },
    }


def test_get_parts_snippet_self_contained(dotnet_extension_self_contained):
    assert dotnet_extension_self_contained.get_parts_snippet() == {
        "dotnet/prereqs": {
            "plugin": "nil",
            "stage-packages": [
                "libicu74",
            ],
        }
    }


@pytest.mark.parametrize(
    "dotnet_version,dotnet_self_contained,content_snap_name",
    [
        ("10.0", True, None),
        ("10.0", False, "dotnet-runtime-100"),
        ("8.0", True, None),
        ("8.0", False, "dotnet-runtime-80"),
        ("7", True, None),
        ("7", False, "dotnet-runtime-70"),
        ("6", True, None),
        ("6.1", False, "dotnet-runtime-61"),
    ],
)
def test_get_application_details(
    dotnet_extension_factory, dotnet_version, dotnet_self_contained, content_snap_name
):
    extension = dotnet_extension_factory(dotnet_version, dotnet_self_contained)
    details = extension.get_application_details

    assert details.dotnet_version == dotnet_version
    assert details.self_contained == dotnet_self_contained
    assert details.content_snap == content_snap_name
