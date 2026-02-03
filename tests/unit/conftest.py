# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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
import io
import textwrap
from collections.abc import Callable
from pathlib import Path
from typing import Any
from unittest.mock import Mock

import craft_application.application
import craft_application.services
import craft_application.util
import pytest
import yaml
from craft_parts import Features, callbacks, plugins
from craft_platforms import BuildInfo, DebianArchitecture
from craft_providers import Executor, Provider
from craft_providers.base import Base
from overrides import override
from pymacaroons import Caveat, Macaroon

from snapcraft import const, models, services
from snapcraft.extensions import extension, register, unregister


@pytest.fixture(autouse=True)
def unregister_callbacks(mocker):
    callbacks.unregister_all()


@pytest.fixture(autouse=True)
def reset_plugins():
    # craft-part modifies a dictionary of plugins that doesn't get reloaded between tests
    # 'unregister_all()' resets to the dictionary to the default value
    plugins.unregister_all()


def _write_yaml(file_path: Path, content: dict[str, Any]) -> None:
    """Write a YAML file from a dict."""
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(
        yaml.safe_dump(content, indent=2, sort_keys=False), encoding="utf-8"
    )


@pytest.fixture
def snapcraft_yaml(new_dir):
    """Return a fixture that can write a snapcraft.yaml."""

    def write_file(
        *, filename: str = "snap/snapcraft.yaml", **kwargs
    ) -> dict[str, Any]:
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
        _write_yaml(Path(filename), content)
        return content

    yield write_file


FAKE_PROJECT_YAML = """\
name: mytest
version: 0.1
summary: Just some test data
description: This is just some test data.
grade: stable
confinement: strict
base: core24
parts:
   part1:
     plugin: nil
"""


@pytest.fixture
def fake_project() -> models.Project:
    with io.StringIO(FAKE_PROJECT_YAML) as project_io:
        return models.Project.unmarshal(
            craft_application.util.safe_yaml_load(project_io)
        )


@pytest.fixture
def fake_extension():
    """Basic extension."""

    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> tuple[str, ...]:
            return ("core22", "core24", "core26")

        @staticmethod
        def get_supported_confinement() -> tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: str | None = None) -> bool:
            return False

        def get_root_snippet(self) -> dict[str, Any]:
            return {"grade": "fake-grade"}

        def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
            return {"plugs": ["fake-plug"]}

        def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
            if plugin_name == "catkin":
                return {}

            return {"after": ["fake-extension/fake-part"]}

        def get_parts_snippet(self) -> dict[str, Any]:
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
        def get_supported_bases() -> tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: str | None = None) -> bool:
            return False

        def get_root_snippet(self) -> dict[str, Any]:
            return {}

        def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
            return {"plugs": ["fake-plug", "fake-plug-extra"]}

        def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
            return {"after": ["fake-extension-extra/fake-part"]}

        def get_parts_snippet(self) -> dict[str, Any]:
            return {"fake-extension-extra/fake-part": {"plugin": "nil"}}

    register("fake-extension-extra", ExtensionImpl)
    yield ExtensionImpl
    unregister("fake-extension-extra")


@pytest.fixture
def fake_extension_invalid_parts():
    class ExtensionImpl(extension.Extension):
        """The test extension implementation."""

        @staticmethod
        def get_supported_bases() -> tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: str | None = None) -> bool:
            return False

        def get_root_snippet(self) -> dict[str, Any]:
            return {"grade": "fake-grade"}

        def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
            return {"plugs": ["fake-plug"]}

        def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
            return {"after": ["fake-extension/fake-part"]}

        def get_parts_snippet(self) -> dict[str, Any]:
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
        def get_supported_bases() -> tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: str | None = None) -> bool:
            return True

        def get_root_snippet(self) -> dict[str, Any]:
            return {}

        def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
            return {}

        def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
            return {}

        def get_parts_snippet(self) -> dict[str, Any]:
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
        def get_supported_bases() -> tuple[str, ...]:
            return ("core22",)

        @staticmethod
        def get_supported_confinement() -> tuple[str, ...]:
            return ("strict",)

        @staticmethod
        def is_experimental(base: str | None = None) -> bool:
            return False

        def get_root_snippet(self) -> dict[str, Any]:
            return {}

        def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
            return {"plugs": ["fake-plug", "fake-plug-extra"]}

        def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
            return {"after": ["fake-extension-extra/fake-part"]}

        def get_parts_snippet(self) -> dict[str, Any]:
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

        @property
        @override
        def name(self) -> str:
            return "fake"

        @property
        @override
        def install_recommendation(self) -> str:
            return "snap"

        def clean_project_environments(self, *, instance_name: str):
            pass

        @classmethod
        def ensure_provider_is_available(cls) -> None:
            pass

        @classmethod
        def is_provider_installed(cls) -> bool:
            return True

        def create_environment(  # type: ignore[reportIncompatibleMethodOverride]
            self, *, instance_name: str
        ):
            yield mock_instance

        @contextlib.contextmanager
        def launched_environment(
            self,
            *,
            project_name: str,
            project_path: Path,
            base_configuration: Base,
            build_base: str | None = None,
            instance_name: str,
            allow_unstable: bool = False,
            shutdown_delay_mins: int | None = None,
            use_base_instance: bool = True,
            prepare_instance: Callable[[Executor], None] | None = None,
        ):
            yield mock_instance

    return FakeProvider()


@pytest.fixture()
def extra_project_params():
    """Configuration fixture for the Project used by the default services."""
    return {"confinement": "devmode"}


@pytest.fixture()
def default_project(extra_project_params):
    from snapcraft.models.project import (  # noqa: PLC0415 (import-outside-top-level)
        Project,
    )

    parts = extra_project_params.pop("parts", {})

    return Project.unmarshal(
        {
            "name": "default",
            "version": "1.0",
            "summary": "default project",
            "description": "default project",
            "base": "core24",
            "grade": "devel",
            "parts": parts,
            "license": "MIT",
            **extra_project_params,
        }
    )


@pytest.fixture()
def fake_services(
    request: pytest.FixtureRequest,
    tmp_path,
    fake_package_service_class,
    fake_lifecycle_service_class,
    fake_remote_build_service_class,
    fake_project_service_class,
    fake_provider_service_class,
    fake_confdb_schemas_service_class,
    project_path,
):
    from snapcraft.application import (  # noqa: PLC0415 (import-outside-top-level)
        APP_METADATA,
    )
    from snapcraft.services import (  # noqa: PLC0415 (import-outside-top-level)
        BuildPlan,
        SnapcraftServiceFactory,
    )

    services.SnapcraftServiceFactory.register("package", fake_package_service_class)
    services.SnapcraftServiceFactory.register("lifecycle", fake_lifecycle_service_class)
    services.SnapcraftServiceFactory.register(
        "remote_build", fake_remote_build_service_class
    )
    services.SnapcraftServiceFactory.register("project", fake_project_service_class)
    services.SnapcraftServiceFactory.register(
        "confdb_schemas", fake_confdb_schemas_service_class
    )
    services.SnapcraftServiceFactory.register("provider", fake_provider_service_class)
    services.SnapcraftServiceFactory.register("build_plan", BuildPlan)

    factory = SnapcraftServiceFactory(app=APP_METADATA)

    factory.update_kwargs(
        "lifecycle", work_dir=tmp_path, cache_dir=tmp_path / "cache", build_plan=[]
    )
    factory.update_kwargs("project", project_dir=project_path)
    factory.update_kwargs("provider", work_dir=tmp_path)

    return factory


@pytest.fixture
def fake_app(fake_services):
    from snapcraft.application import (  # noqa: PLC0415 (import-outside-top-level)
        APP_METADATA,
        Snapcraft,
    )
    from snapcraft.cli import (  # noqa: PLC0415 (import-outside-top-level)
        COMMAND_GROUPS,
        CORE24_LIFECYCLE_COMMAND_GROUP,
    )

    app = Snapcraft(app=APP_METADATA, services=fake_services)

    for group in [CORE24_LIFECYCLE_COMMAND_GROUP, *COMMAND_GROUPS]:
        app.add_command_group(group.name, group.commands)

    return app


@pytest.fixture()
def fake_app_config(fake_app) -> dict[str, Any]:
    return fake_app.app_config


def fake_build_info(fake_base):
    arch = DebianArchitecture.from_host()
    return BuildInfo(
        platform=str(arch),
        build_on=arch,
        build_for=arch,
        build_base=fake_base,
    )


@pytest.fixture
def setup_project(mocker, project_path):
    """A helper function to set up the project and build plan."""

    def _setup_services(project_services, project_data, *, write_project: bool = False):
        from snapcraft import models  # noqa: PLC0415 (import-outside-top-level)

        if project_data.get("base") in (b.value for b in const.UnstableBase):
            project_data |= {
                "build-base": "devel",
                "grade": "devel",
            }

        if write_project:
            _write_yaml(
                file_path=project_path / "snapcraft.yaml",
                content=project_data,
            )

        mocker.patch.object(
            project_services.get("project"),
            "_load_raw_project",
            return_value=project_data,
        )
        project_services.get("project").configure(platform=None, build_for=None)

        return models.Project.unmarshal(project_data)

    return _setup_services


@pytest.fixture()
def fake_lifecycle_service_class(in_project_path):
    from snapcraft.services import Lifecycle  # noqa: PLC0415 (import-outside-top-level)

    class FakeLifecycleService(Lifecycle):
        def __init__(
            self,
            app: craft_application.application.AppMetadata,
            services: craft_application.services.ServiceFactory,
            **kwargs: Any,
        ):
            kwargs.pop("build_plan", None)  # We'll use ours
            super().__init__(
                app,
                services,
                work_dir=kwargs.pop("work_dir", in_project_path / "work"),
                cache_dir=kwargs.pop("cache_dir", in_project_path / "cache"),
                platform=None,
                **kwargs,
            )

    return FakeLifecycleService


@pytest.fixture()
def fake_provider_service_class(project_path):
    from snapcraft.services import Provider  # noqa: PLC0415 (import-outside-top-level)

    class FakeProviderService(Provider):
        def __init__(
            self,
            app: craft_application.application.AppMetadata,
            services: craft_application.services.ServiceFactory,
            work_dir: Path,
        ):
            super().__init__(
                app,
                services,
                work_dir=project_path,
            )

    return FakeProviderService


@pytest.fixture
def fake_project_service_class(fake_project) -> type[services.Project]:
    class FakeProjectService(services.Project):
        # This is a final method, but we're overriding it here for convenience when
        # doing internal testing.
        def _load_raw_project(self):  # type: ignore[reportIncompatibleMethodOverride]
            return fake_project.marshal()

        # Don't care if the project file exists during this testing.
        # Silencing B019 because we're replicating an inherited method.
        @override
        def resolve_project_file_path(self) -> Path:
            return (self._project_dir / f"{self._app.name}.yaml").resolve()

        def set(self, value: models.Project) -> None:
            """Set the project model. Only for use during testing!"""
            self._project_model = value
            # this is from craft-application, why does pyright only flag this in snapcraft?
            self._platform = next(iter(value.platforms))  # type: ignore[reportCallIssue, reportArgumentType]
            self._build_for = value.platforms[self._platform].build_for[0]  # type: ignore[reportOptionalSubscript]

    return FakeProjectService


@pytest.fixture()
def fake_package_service_class(default_project, snapcraft_yaml, tmp_path):
    from snapcraft.services import Package  # noqa: PLC0415 (import-outside-top-level)

    class FakePackageService(Package):
        pass

    return FakePackageService


@pytest.fixture()
def fake_remote_build_service_class(mocker):
    import lazr.restfulclient.resource  # noqa: PLC0415 (import-outside-top-level)
    from craft_application import (  # noqa: PLC0415 (import-outside-top-level)
        AppMetadata,
    )
    from craft_application.launchpad.models import (  # noqa: PLC0415 (import-outside-top-level)
        SnapRecipe,
    )

    me = Mock(lazr.restfulclient.resource.Entry)
    me.name = "craft_test_user"

    class FakeRemoteBuildService(craft_application.services.RemoteBuildService):
        """Fake remote build service with snap recipe."""

        RecipeClass = SnapRecipe

        @override
        def __init__(
            self, app: AppMetadata, services: craft_application.services.ServiceFactory
        ) -> None:
            super().__init__(app=app, services=services)
            self._is_setup = True

    # The login should not do anything
    mocker.patch("craft_application.launchpad.Launchpad.anonymous")
    mocker.patch("craft_application.launchpad.Launchpad.login")

    return FakeRemoteBuildService


@pytest.fixture()
def fake_confdb_schemas_service_class(mocker):
    from snapcraft.services import (  # noqa: PLC0415 (import-outside-top-level)
        ConfdbSchemas,
    )

    class FakeConfdbSchemasService(ConfdbSchemas):
        def setup(self) -> None:
            """Application-specific service setup."""
            self._store_client = mocker.patch(
                "snapcraft.store.StoreClientCLI", autospec=True
            )
            super().setup()

    return FakeConfdbSchemasService


@pytest.fixture()
def fake_confdb_schema_assertion():
    """Returns a fake confdb-schema assertion with required fields."""
    from snapcraft.models import (  # noqa: PLC0415 (import-outside-top-level)
        ConfdbSchemaAssertion,
    )

    def _fake_confdb_schema_assertion(**kwargs) -> ConfdbSchemaAssertion:
        return ConfdbSchemaAssertion.unmarshal(
            {
                "account_id": "test-account-id",
                "authority_id": "test-authority-id",
                "name": "test-confdb",
                "timestamp": "2024-01-01T10:20:30Z",
                "type": "confdb-schema",
                "views": {
                    "wifi-setup": {
                        "rules": [
                            {
                                "access": "read-write",
                                "request": "ssids",
                                "storage": "wifi.ssids",
                            }
                        ]
                    }
                },
                **kwargs,
            }
        )

    return _fake_confdb_schema_assertion


@pytest.fixture()
def enable_partitions_feature():
    """Resets the partitions feature in craft-parts."""
    assert Features().enable_partitions is False
    Features.reset()
    Features(enable_partitions=True)
    yield
    Features.reset()
