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

from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import pytest
import yaml

from snapcraft.extensions import extension, register, unregister


@pytest.fixture
def snapcraft_yaml(new_dir):
    """Return a fixture that can write a snapcraft.yaml."""

    def write_file(
        *, filename: str = "snap/snapcraft.yaml", **kwargs
    ) -> Dict[str, Any]:
        content = {
            "name": "mytest",
            "version": "0.1",
            "summary": "Just some test data",
            "description": "This is just some test data.",
            "grade": "stable",
            "confinement": "strict",
            "parts": {
                "part1": {
                    "plugin": "nil",
                }
            },
            **kwargs,
        }
        yaml_path = Path(filename)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(
            yaml.safe_dump(content, indent=2, sort_keys=False), encoding="utf-8"
        )
        return content

    yield write_file


@pytest.fixture
def fake_extension():
    """Basic extension."""

    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: Optional[str] = None) -> bool:
            return False

        def get_root_snippet(self) -> Dict[str, Any]:
            return {"grade": "fake-grade"}

        def get_app_snippet(self) -> Dict[str, Any]:
            return {"plugs": ["fake-plug"]}

        def get_part_snippet(self) -> Dict[str, Any]:
            return {"after": ["fake-extension/fake-part"]}

        def get_parts_snippet(self) -> Dict[str, Any]:
            return {"fake-extension/fake-part": {"plugin": "nil"}}

    register("fake-extension", ExtensionImpl)
    yield ExtensionImpl
    unregister("fake-extension")


@pytest.fixture
def fake_extension_extra():
    """A variation of fake_extension with some conflicts and new code."""

    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: Optional[str] = None) -> bool:
            return False

        def get_root_snippet(self) -> Dict[str, Any]:
            return {}

        def get_app_snippet(self) -> Dict[str, Any]:
            return {"plugs": ["fake-plug", "fake-plug-extra"]}

        def get_part_snippet(self) -> Dict[str, Any]:
            return {"after": ["fake-extension-extra/fake-part"]}

        def get_parts_snippet(self) -> Dict[str, Any]:
            return {"fake-extension-extra/fake-part": {"plugin": "nil"}}

    register("fake-extension-extra", ExtensionImpl)
    yield ExtensionImpl
    unregister("fake-extension-extra")


@pytest.fixture
def fake_extension_invalid_parts():
    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: Optional[str] = None) -> bool:
            return False

        def get_root_snippet(self) -> Dict[str, Any]:
            return {"grade": "fake-grade"}

        def get_app_snippet(self) -> Dict[str, Any]:
            return {"plugs": ["fake-plug"]}

        def get_part_snippet(self) -> Dict[str, Any]:
            return {"after": ["fake-extension/fake-part"]}

        def get_parts_snippet(self) -> Dict[str, Any]:
            return {"fake-part": {"plugin": "nil"}, "fake-part-2": {"plugin": "nil"}}

    register("fake-extension-invalid-parts", ExtensionImpl)
    yield ExtensionImpl
    unregister("fake-extension-invalid-parts")


@pytest.fixture
def fake_extension_experimental():
    """Basic extension."""

    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: Optional[str] = None) -> bool:
            return True

        def get_root_snippet(self) -> Dict[str, Any]:
            return {}

        def get_app_snippet(self) -> Dict[str, Any]:
            return {}

        def get_part_snippet(self) -> Dict[str, Any]:
            return {}

        def get_parts_snippet(self) -> Dict[str, Any]:
            return {}

    register("fake-extension-experimental", ExtensionImpl)
    yield ExtensionImpl
    unregister("fake-extension-experimental")


@pytest.fixture
def fake_extension_name_from_legacy():
    """A fake_extension variant with a name collision with legacy."""

    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: Optional[str] = None) -> bool:
            return False

        def get_root_snippet(self) -> Dict[str, Any]:
            return {}

        def get_app_snippet(self) -> Dict[str, Any]:
            return {"plugs": ["fake-plug", "fake-plug-extra"]}

        def get_part_snippet(self) -> Dict[str, Any]:
            return {"after": ["fake-extension-extra/fake-part"]}

        def get_parts_snippet(self) -> Dict[str, Any]:
            return {"fake-extension-extra/fake-part": {"plugin": "nil"}}

    register("ros2-foxy", ExtensionImpl)
    yield ExtensionImpl
    unregister("ros2-foxy")
