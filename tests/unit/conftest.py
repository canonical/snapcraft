# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

import base64
import contextlib
import textwrap
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from unittest.mock import Mock

import pytest
import yaml
from craft_providers import Executor, Provider
from craft_providers.base import Base
from pymacaroons import Caveat, Macaroon

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

    yield ExtensionImpl


@pytest.fixture
def fake_client(mocker):
    """Forces get_client to return a fake craft_store.BaseClient"""
    client = mocker.patch("craft_store.BaseClient", autospec=True)
    mocker.patch("snapcraft.store.client.get_client", return_value=client)
    return client


@pytest.fixture
def fake_confirmation_prompt(mocker):
    """Fake the confirmation prompt."""
    return mocker.patch(
        "snapcraft.utils.confirm_with_user", return_value=False, autospec=True
    )


@pytest.fixture
def root_macaroon():
    return Macaroon(
        location="fake-server.com",
        signature="d9533461d7835e4851c7e3b639144406cf768597dea6e133232fbd2385a5c050",
        caveats=[
            Caveat(
                caveat_id="1234567890",
                location="fake-sso.com",
                verification_key_id="1234567890",
            )
        ],
    ).serialize()


@pytest.fixture
def discharged_macaroon():
    return Macaroon(
        location="fake-server.com",
        signature="d9533461d7835e4851c7e3b639122406cf768597dea6e133232fbd2385a5c050",
    ).serialize()


@pytest.fixture(params=["encode", "no-encode"])
def legacy_config_credentials(request):
    config = textwrap.dedent(
        f"""\
        [login.ubuntu.com]
        macaroon={root_macaroon}
        unbound_discharge={discharged_macaroon}
        """
    )

    if request.param == "encode":
        return base64.b64encode(config.encode()).decode()

    if request.param == "no-encode":
        return config

    raise RuntimeError("unhandled param")


@pytest.fixture
def legacy_config_path(
    monkeypatch, new_dir, root_macaroon, discharged_macaroon, legacy_config_credentials
):
    config_file = new_dir / "snapcraft.cfg"
    monkeypatch.setattr(
        "snapcraft.store._legacy_account.LegacyUbuntuOne.CONFIG_PATH",
        config_file,
    )

    config_file.write_text(legacy_config_credentials)

    return config_file


@pytest.fixture
def mock_instance():
    """Provide a mock instance (Executor)."""
    yield Mock(spec=Executor)


@pytest.fixture(autouse=True)
def fake_provider(mock_instance):
    """Fixture to provide a minimal fake provider."""

    class FakeProvider(Provider):
        """Fake provider."""

        def clean_project_environments(self, *, instance_name: str):
            pass

        @classmethod
        def ensure_provider_is_available(cls) -> None:
            pass

        @classmethod
        def is_provider_installed(cls) -> bool:
            return True

        def create_environment(self, *, instance_name: str):
            yield mock_instance

        @contextlib.contextmanager
        def launched_environment(
            self,
            *,
            project_name: str,
            project_path: Path,
            base_configuration: Base,
            build_base: str,
            instance_name: str,
            allow_unstable: bool = False,
        ):
            yield mock_instance

    return FakeProvider()
