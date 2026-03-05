# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""GPUExtension base class for extensions that integrate GPU support snaps."""

from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir


class GPUExtension(Extension):
    """An extension base class that integrates GPU support snaps.

    For core26, integrates the gpu-2604 provider snaps.
    For core24, integrates the gpu-2404 provider snaps.
    For core22, integrates the graphics-core22 provider snaps.

    Handles GPU-specific integration points shared across desktop extensions:

    - Adding the appropriate GPU/graphics plug in ``get_root_snippet``.
    - Prepending the GPU/graphics wrapper to the command chain in ``get_app_snippet``.
    - Adding GPU/graphics-specific layouts in ``get_root_snippet``.

    Methods only act when the base is ``core22``, ``core24``, or ``core26``;
    for other bases they raise AssertionError to prevent subclasses from being instantiated.

    Subclasses should call ``super()`` when overriding these methods to inherit the
    GPU integration and build on top of it.
    """

    _GRAPHICS_CORE22_PLUG: dict[str, Any] = {
        "graphics-core22": {
            "interface": "content",
            "target": "$SNAP/graphics-core22",
            "default-provider": "mesa-core22",
        },
    }

    _GRAPHICS_CORE22_LAYOUTS: dict[str, Any] = {
        "/usr/share/libdrm": {"bind": "$SNAP/graphics-core22/libdrm"},
        "/usr/share/drirc.d": {"symlink": "$SNAP/graphics-core22/drirc.d"},
        "/usr/share/X11/XErrorDB": {"symlink": "$SNAP/graphics-core22/X11/XErrorDB"},
        "/usr/share/X11/locale": {"symlink": "$SNAP/graphics-core22/X11/locale"},
    }

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

    _GPU_2604_PLUG: dict[str, Any] = {
        "gpu-2604": {
            "interface": "content",
            "target": "$SNAP/gpu-2604",
            "default-provider": "mesa-2604",
        },
    }

    _GPU_2604_LAYOUTS: dict[str, Any] = {
        "/usr/share/X11/XErrorDB": {"symlink": "$SNAP/gpu-2604/X11/XErrorDB"},
    }

    @staticmethod
    @overrides
    def get_supported_bases() -> tuple[str, ...]:
        """Return the tuple of supported bases."""
        return ("core22", "core24", "core26")

    @staticmethod
    @overrides
    def get_supported_confinement() -> tuple[str, ...]:
        """Return the tuple of supported confinement modes."""
        return ("strict", "devmode")

    @staticmethod
    @overrides
    def is_experimental(base: str | None) -> bool:
        """Return whether this extension is experimental for the given base."""
        return base == "core26"

    @overrides
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        """Return the GPU/graphics wrapper command-chain entry."""
        base = self.yaml_data["base"]
        if base == "core22":
            return {"command-chain": ["snap/command-chain/graphics-core22-wrapper"]}
        if base == "core24":
            return {"command-chain": ["snap/command-chain/gpu-2404-wrapper"]}
        if base == "core26":
            return {"command-chain": ["snap/command-chain/gpu-2604-wrapper"]}
        raise AssertionError(f"Unsupported base: {base}")

    @overrides
    def get_root_snippet(self) -> dict[str, Any]:
        """Return the GPU/graphics plug and layouts."""
        base = self.yaml_data["base"]
        if base == "core22":
            return {
                "plugs": dict(self._GRAPHICS_CORE22_PLUG),
                "layout": dict(self._GRAPHICS_CORE22_LAYOUTS),
            }
        if base == "core24":
            return {
                "plugs": dict(self._GPU_2404_PLUG),
                "layout": dict(self._GPU_2404_LAYOUTS),
            }
        if base == "core26":
            return {
                "plugs": dict(self._GPU_2604_PLUG),
                "layout": dict(self._GPU_2604_LAYOUTS),
            }
        raise AssertionError(f"Unsupported base: {base}")

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        """Return an empty dict as GPU extension doesn't modify existing parts."""
        return {}

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        """Return the parts for the GPU/graphics wrapper."""
        base = self.yaml_data["base"]
        source = get_extensions_data_dir() / "gpu" / "command-chain"

        if base == "core22":
            return {
                "gpu/wrapper": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": ["GPU_INTERFACE=graphics-core22"],
                },
                # TODO: remove this part when https://github.com/canonical/snapcraft/issues/6066 is resolved
                # and Snapcraft prunes the consumer snap itself
                "gpu/cleanup": {
                    "after": list(self.yaml_data.get("parts", {}).keys()),
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
        if base == "core24":
            return {
                "gpu/wrapper": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": ["GPU_INTERFACE=gpu-2404"],
                },
                # TODO: remove this part when https://github.com/canonical/snapcraft/issues/6066 is resolved
                # and Snapcraft prunes the consumer snap itself
                "gpu/cleanup": {
                    "after": list(self.yaml_data.get("parts", {}).keys()),
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
        if base == "core26":
            return {
                "gpu/wrapper": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": ["GPU_INTERFACE=gpu-2604"],
                },
                # TODO: remove this part when https://github.com/canonical/snapcraft/issues/6066 is resolved
                # and Snapcraft prunes the consumer snap itself
                "gpu/cleanup": {
                    "after": list(self.yaml_data.get("parts", {}).keys()),
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
        raise AssertionError(f"Unsupported base: {base}")
