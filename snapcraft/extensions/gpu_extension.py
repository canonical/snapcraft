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

"""GPUExtension base class for extensions that integrate the gpu-2404 snap."""

from typing import Any

from typing_extensions import override

from .extension import Extension, get_extensions_data_dir


class GPUExtension(Extension):
    """An extension base class that integrates the gpu-2404 snap for core24.

    Handles the gpu-2404-specific integration points shared across desktop extensions:

    - Prepending the gpu-2404-wrapper to the command chain in ``get_app_snippet``.
    - Adding the gpu-2404 plug and layouts in ``get_root_snippet``.

    Currently only supports the core24 base and raises ``AssertionError`` on unsupported ones.

    Subclasses should call ``super()`` when overriding these methods to inherit the
    gpu-2404 integration and build on top of it.
    """

    _GPU_2404_PLUG: dict[str, Any] = {
        "gpu-2404": {
            "interface": "content",
            "target": "$SNAP/gpu-2404",
            "default-provider": "mesa-2404",
        },
    }

    _GPU_2404_LAYOUTS: dict[str, Any] = {
        "/usr/share/libdrm": {"bind": "$SNAP/gpu-2404/libdrm"},
        "/usr/share/drirc.d": {"symlink": "$SNAP/gpu-2404/drirc.d"},
        "/usr/share/X11/XErrorDB": {"symlink": "$SNAP/gpu-2404/X11/XErrorDB"},
    }

    @staticmethod
    @override
    def get_supported_bases() -> tuple[str, ...]:
        """Return the tuple of supported bases."""
        return ("core24",)

    @staticmethod
    @override
    def get_supported_confinement() -> tuple[str, ...]:
        """Return the tuple of supported confinement modes."""
        return ("strict", "devmode")

    @staticmethod
    @override
    def is_experimental(base: str | None) -> bool:
        """Return whether this extension is experimental for the given base."""
        return False

    @override
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        """Return the gpu-2404-wrapper command-chain entry for core24."""
        if self.yaml_data["base"] == "core24":
            return {"command-chain": ["snap/command-chain/gpu-2404-wrapper"]}
        raise AssertionError(f"Unsupported base: {self.yaml_data['base']}")

    @override
    def get_root_snippet(self) -> dict[str, Any]:
        """Return the gpu-2404 plug and layouts for core24."""
        if self.yaml_data["base"] == "core24":
            return {
                "plugs": dict(self._GPU_2404_PLUG),
                "layout": dict(self._GPU_2404_LAYOUTS),
            }
        raise AssertionError(f"Unsupported base: {self.yaml_data['base']}")

    @override
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        """Return an empty dict as GPU extension doesn't modify existing parts."""
        return {}

    @override
    def get_parts_snippet(self) -> dict[str, Any]:
        """Return the part for the gpu-2404-wrapper for core24."""
        source = get_extensions_data_dir() / "gpu" / "command-chain"

        if self.yaml_data["base"] == "core24":
            return {
                "gpu/wrapper": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": ["GPU_WRAPPER=gpu-2404-wrapper"],
                },
            }
        raise AssertionError(f"Unsupported base: {self.yaml_data['base']}")
