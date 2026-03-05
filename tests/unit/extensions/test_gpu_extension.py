# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

from snapcraft.extensions import gpu_extension
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def gpu_extension_core24():
    return gpu_extension.GPUExtension(
        yaml_data={"base": "core24", "parts": {"other_part": {}}},
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def gpu_extension_core22():
    return gpu_extension.GPUExtension(
        yaml_data={"base": "core22", "parts": {"other_part": {}}},
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def gpu_extension_core26():
    return gpu_extension.GPUExtension(
        yaml_data={"base": "core26", "parts": {"other_part": {}}},
        arch="amd64",
        target_arch="amd64",
    )


####################
# GPU Extension    #
####################


def test_get_supported_bases():
    """Test that GPU extension supports core22, core24, and core26."""
    assert gpu_extension.GPUExtension.get_supported_bases() == (
        "core22",
        "core24",
        "core26",
    )


def test_get_supported_confinement():
    """Test that GPU extension supports strict and devmode confinement."""
    assert gpu_extension.GPUExtension.get_supported_confinement() == (
        "strict",
        "devmode",
    )


@pytest.mark.parametrize(
    ("base", "experimental"), [("core22", False), ("core24", False), ("core26", True)]
)
def test_is_experimental(base, experimental):
    """Test that GPU extension is not experimental."""
    assert gpu_extension.GPUExtension.is_experimental(base=base) is experimental


def test_get_app_snippet(gpu_extension_core24):
    """Test that GPU extension (core24) adds gpu-2404-wrapper to command-chain."""
    assert gpu_extension_core24.get_app_snippet(app_name="test-app") == {
        "command-chain": ["snap/command-chain/gpu-2404-wrapper"],
    }


def test_get_app_snippet_core22(gpu_extension_core22):
    """Test that GPU extension (core22) adds graphics-core22-wrapper to command-chain."""
    assert gpu_extension_core22.get_app_snippet(app_name="test-app") == {
        "command-chain": ["snap/command-chain/graphics-core22-wrapper"],
    }


def test_get_app_snippet_core26(gpu_extension_core26):
    """Test that GPU extension (core26) adds gpu-2604-wrapper to command-chain."""
    assert gpu_extension_core26.get_app_snippet(app_name="test-app") == {
        "command-chain": ["snap/command-chain/gpu-2604-wrapper"],
    }


def test_get_root_snippet(gpu_extension_core24):
    """Test that GPU extension (core24) adds gpu-2404 plug and X11 error database layout."""
    assert gpu_extension_core24.get_root_snippet() == {
        "plugs": {
            "gpu-2404": {
                "interface": "content",
                "target": "$SNAP/gpu-2404",
                "default-provider": "mesa-2404",
            },
        },
        "layout": {
            "/usr/share/libdrm": {
                "bind": "$SNAP/gpu-2404/libdrm",
            },
            "/usr/share/drirc.d": {
                "symlink": "$SNAP/gpu-2404/drirc.d",
            },
            "/usr/share/X11/XErrorDB": {
                "symlink": "$SNAP/gpu-2404/X11/XErrorDB",
            },
        },
    }


def test_get_root_snippet_core22(gpu_extension_core22):
    """Test that GPU extension (core22) adds graphics-core22 plug and layouts."""
    assert gpu_extension_core22.get_root_snippet() == {
        "plugs": {
            "graphics-core22": {
                "interface": "content",
                "target": "$SNAP/graphics-core22",
                "default-provider": "mesa-core22",
            },
        },
        "layout": {
            "/usr/share/libdrm": {
                "bind": "$SNAP/graphics-core22/libdrm",
            },
            "/usr/share/drirc.d": {
                "symlink": "$SNAP/graphics-core22/drirc.d",
            },
            "/usr/share/X11/XErrorDB": {
                "symlink": "$SNAP/graphics-core22/X11/XErrorDB",
            },
            "/usr/share/X11/locale": {
                "symlink": "$SNAP/graphics-core22/X11/locale",
            },
        },
    }


def test_get_root_snippet_core26(gpu_extension_core26):
    """Test that GPU extension (core26) adds gpu-2604 plug and X11 layout only."""
    assert gpu_extension_core26.get_root_snippet() == {
        "plugs": {
            "gpu-2604": {
                "interface": "content",
                "target": "$SNAP/gpu-2604",
                "default-provider": "mesa-2604",
            },
        },
        "layout": {
            "/usr/share/X11/XErrorDB": {
                "symlink": "$SNAP/gpu-2604/X11/XErrorDB",
            },
        },
    }


def test_get_part_snippet(gpu_extension_core24):
    """Test that GPU extension doesn't modify existing parts."""
    assert gpu_extension_core24.get_part_snippet(plugin_name="make") == {}


def test_get_parts_snippet(gpu_extension_core24):
    """Test that GPU extension (core24) adds gpu/wrapper part."""
    assert gpu_extension_core24.get_parts_snippet() == {
        "gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=gpu-2404"],
        },
        "gpu/cleanup": {
            "after": ["other_part"],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n"
                "${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-core24\n"
                "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                'mkdir -p "${CRAFT_PRIME}/gpu-2404"'
            ),
        },
    }


def test_get_parts_snippet_core22(gpu_extension_core22):
    """Test that GPU extension (core22) adds gpu/wrapper part."""
    assert gpu_extension_core22.get_parts_snippet() == {
        "gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=graphics-core22"],
        },
        "gpu/cleanup": {
            "after": ["other_part"],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n"
                "${CRAFT_PART_SRC}/bin/graphics-core22-cleanup mesa-core22\n"
                "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                'mkdir -p "${CRAFT_PRIME}/gpu-2404"'
            ),
        },
    }


def test_get_parts_snippet_core26(gpu_extension_core26):
    """Test that GPU extension (core26) adds gpu/wrapper and gpu/cleanup parts."""
    assert gpu_extension_core26.get_parts_snippet() == {
        "gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=gpu-2604"],
        },
        "gpu/cleanup": {
            "after": ["other_part"],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n"
                "${CRAFT_PART_SRC}/bin/gpu-2604-cleanup mesa-2604\n"
                "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                'mkdir -p "${CRAFT_PRIME}/gpu-2604"'
            ),
        },
    }


def test_app_snippet_unsupported_base():
    """Test that GPU extension raises error for unsupported base."""
    extension = gpu_extension.GPUExtension(
        yaml_data={"base": "core20", "parts": {}}, arch="amd64", target_arch="amd64"
    )
    with pytest.raises(AssertionError, match="Unsupported base: core20"):
        extension.get_app_snippet(app_name="test")


def test_root_snippet_unsupported_base():
    """Test that GPU extension raises error for unsupported base."""
    extension = gpu_extension.GPUExtension(
        yaml_data={"base": "core20", "parts": {}}, arch="amd64", target_arch="amd64"
    )
    with pytest.raises(AssertionError, match="Unsupported base: core20"):
        extension.get_root_snippet()


def test_parts_snippet_unsupported_base():
    """Test that GPU extension raises error for unsupported base."""
    extension = gpu_extension.GPUExtension(
        yaml_data={"base": "core20", "parts": {}}, arch="amd64", target_arch="amd64"
    )
    with pytest.raises(AssertionError, match="Unsupported base: core20"):
        extension.get_parts_snippet()
