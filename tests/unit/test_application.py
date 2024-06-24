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
"""Unit tests for application classes."""

import json
import os
import sys
from textwrap import dedent

import craft_cli
import craft_parts.plugins
import craft_store
import pytest
import yaml
from craft_application import util
from craft_parts.packages import snaps
from craft_providers import bases

from snapcraft import application, const, services
from snapcraft.commands import PackCommand
from snapcraft.errors import ClassicFallback, SnapcraftError
from snapcraft.models.project import Architecture


@pytest.fixture(
    params=[
        ["amd64"],
        ["riscv64"],
        ["amd64", "riscv64", "s390x"],
        [Architecture(build_on="amd64", build_for="riscv64")],
    ]
)
def architectures(request):
    return request.param


@pytest.fixture()
def mock_confirm(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.confirm_with_user", return_value=True
    )


@pytest.fixture()
def mock_remote_build_run(mocker):
    _mock_remote_build_run = mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run"
    )
    return _mock_remote_build_run


@pytest.fixture()
def mock_run_legacy(mocker):
    return mocker.patch("snapcraft_legacy.cli.legacy.legacy_run")


@pytest.fixture()
def mock_remote_build_argv(mocker):
    """Mock `snapcraft remote-build` cli."""
    return mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])


@pytest.mark.parametrize("env_vars", application.MAPPED_ENV_VARS.items())
def test_application_map_build_on_env_var(monkeypatch, env_vars):
    """Test that instantiating the Snapcraft application class will set the value of the
    SNAPCRAFT_* environment variables to CRAFT_*.
    """
    craft_var = env_vars[0]
    snapcraft_var = env_vars[1]
    env_val = "woop"

    monkeypatch.setenv(snapcraft_var, env_val)
    assert os.getenv(craft_var) is None

    snapcraft_services = services.SnapcraftServiceFactory(app=application.APP_METADATA)
    application.Snapcraft(app=application.APP_METADATA, services=snapcraft_services)

    assert os.getenv(craft_var) == env_val
    assert os.getenv(snapcraft_var) == env_val


def test_application_map_log_verbosity_env_var(monkeypatch):
    """Test that instantiating the Snapcraft application class will set the value of the
    SNAPCRAFT_VERBOSITY_LEVEL environment variables to CRAFT_VERBOSITY_LEVEL.
    """
    old_emit_level = craft_cli.emit.get_mode()

    monkeypatch.setenv("SNAPCRAFT_VERBOSITY_LEVEL", "TRACE")
    assert os.getenv("CRAFT_VERBOSITY_LEVEL") is None

    snapcraft_services = services.SnapcraftServiceFactory(app=application.APP_METADATA)
    app = application.Snapcraft(
        app=application.APP_METADATA, services=snapcraft_services
    )

    app._setup_logging()

    assert os.getenv("SNAPCRAFT_VERBOSITY_LEVEL") == "TRACE"
    assert os.getenv("CRAFT_VERBOSITY_LEVEL") == "TRACE"

    assert craft_cli.emit.get_mode() == craft_cli.EmitterMode.TRACE

    craft_cli.emit.set_mode(old_emit_level)


@pytest.fixture()
def extension_source(default_project):
    source = default_project.marshal()
    source["confinement"] = "strict"
    source["apps"] = {
        "app1": {
            "command": "app1",
            "extensions": ["fake-extension"],
        }
    }
    return source


@pytest.mark.usefixtures("fake_extension")
def test_application_expand_extensions(emitter, monkeypatch, extension_source, new_dir):
    monkeypatch.setenv("CRAFT_DEBUG", "1")

    (new_dir / "snap").mkdir()
    (new_dir / "snap/snapcraft.yaml").write_text(json.dumps(extension_source))

    monkeypatch.setattr("sys.argv", ["snapcraft", "expand-extensions"])
    application.main()
    emitter.assert_message(
        dedent(
            """\
            name: default
            version: '1.0'
            summary: default project
            description: default project
            base: core24
            license: MIT
            parts:
                fake-extension/fake-part:
                    plugin: nil
            confinement: strict
            grade: devel
            apps:
                app1:
                    command: app1
                    plugs:
                    - fake-plug
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_application_extra_yaml_transforms(
    monkeypatch, extension_source, new_dir, emitter
):
    """Test that extra_yaml_transforms applies root keywords and expands extensions."""
    monkeypatch.setenv("CRAFT_DEBUG", "1")
    extension_source["build-packages"] = [{"to s390x": "test-package"}]
    extension_source["build-snaps"] = [{"to s390x": "test-snap"}]

    project_path = new_dir / "snap/snapcraft.yaml"
    (new_dir / "snap").mkdir()
    project_path.write_text(json.dumps(extension_source))

    # Calling a lifecycle command will create a Project. Creating a Project
    # without applying the extensions will fail because the "extensions" field
    # will still be present on the yaml data, so it's enough to run "pull".
    monkeypatch.setattr(
        "sys.argv", ["snapcraft", "pull", "--destructive-mode", "--build-for", "s390x"]
    )
    app = application.create_app()
    app.run()

    project = app.get_project()
    assert "fake-extension/fake-part" in project.parts
    assert project.parts["snapcraft/core"]["build-packages"] == ["test-package"]
    assert project.parts["snapcraft/core"]["build-snaps"] == ["test-snap"]


def test_application_managed_core20_fallback(monkeypatch, new_dir, mocker):
    monkeypatch.setenv("CRAFT_DEBUG", "1")
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "managed-host")

    (new_dir / "snap").mkdir()

    mock_legacy_run = mocker.patch("snapcraft_legacy.cli.legacy.legacy_run")
    mock_create_app = mocker.patch.object(application, "create_app")

    application.main()

    mock_create_app.assert_not_called()
    mock_legacy_run.assert_called()


PARSE_INFO_PROJECT = dedent(
    """\
    name: parse-info-project
    base: core24
    build-base: devel

    grade: devel
    confinement: strict
    adopt-info: parse-info-part

    parts:
      parse-info-part:
        plugin: nil
        source: .
        parse-info: [usr/share/metainfo/app.metainfo.xml]
        override-build: |
          craftctl default
          mkdir -p ${CRAFT_PART_INSTALL}/usr/share/metainfo
          cp metainfo.xml ${CRAFT_PART_INSTALL}/usr/share/metainfo/app.metainfo.xml
"""
)


def test_get_project_parse_info(new_dir):
    """Test that parse-info data is correctly extracted and stored when loading
    the project from a YAML file."""
    snap_dir = new_dir / "snap"
    snap_dir.mkdir()
    project_yaml = snap_dir / "snapcraft.yaml"
    project_yaml.write_text(PARSE_INFO_PROJECT)

    app = application.create_app()
    assert app._parse_info == {}

    _project = app.get_project()
    assert app._parse_info == {
        "parse-info-part": ["usr/share/metainfo/app.metainfo.xml"]
    }


APPSTREAM_CONTENTS = dedent(
    """\
    <?xml version="1.0" encoding="UTF-8"?>
    <!-- Some Comment -->
    <component type="desktop-application">
      <id>io.snapcraft.appstream</id>
      <metadata_license>FSFAP</metadata_license>
      <project_license>GPL-2.0+</project_license>
      <name>Sample app</name>
      <summary>Sample summary</summary>

      <description><p>Sample description</p></description>

      <releases>
        <release version="1.2.3" date="2020-01-01">
          <description>
            <p>Initial release.</p>
          </description>
        </release>
      </releases>
    </component>
    """
)


def test_parse_info_integrated(monkeypatch, mocker, new_dir):
    # Pretend this is an Ubuntu 24.04 system, to match the project's build-base
    mocker.patch.object(
        util, "get_host_base", return_value=bases.BaseName("ubuntu", "24.04")
    )

    # Mock the installation of the core24 snap, as it can currently fail due
    # to network issues and it's not necessary for the test
    mocker.patch.object(snaps, "install_snaps")

    snap_dir = new_dir / "snap"
    snap_dir.mkdir()

    project_yaml = snap_dir / "snapcraft.yaml"
    project_yaml.write_text(PARSE_INFO_PROJECT)

    metainfo_file = new_dir / "metainfo.xml"
    metainfo_file.write_text(APPSTREAM_CONTENTS)

    monkeypatch.setattr("sys.argv", ["snapcraft", "prime", "--destructive-mode"])
    app = application.create_app()
    app.run()

    # Check for the parsed data directly in the generated snap.yaml
    snap_file = new_dir / "prime/meta/snap.yaml"
    snap_yaml = yaml.safe_load(snap_file.read_text())

    assert snap_yaml["summary"] == "Sample summary"
    assert snap_yaml["description"] == "Sample description"
    assert snap_yaml["version"] == "1.2.3"


def test_application_plugins():
    app = application.create_app()
    plugins = app._get_app_plugins()

    # Just do some sanity checks.
    assert "python" in plugins
    assert "kernel" not in plugins


@pytest.mark.parametrize(
    ("base", "build_base"),
    [
        ("core20", None),
        ("core20", "core20"),
        ("core20", "devel"),
        ("core22", None),
        ("core22", "core22"),
        ("core22", "devel"),
    ],
)
def test_application_dotnet_registered(base, build_base, snapcraft_yaml):
    """dotnet plugin is enabled for core22."""
    snapcraft_yaml(base=base, build_base=build_base)
    app = application.create_app()

    app._register_default_plugins()

    assert "dotnet" in craft_parts.plugins.get_registered_plugins()


@pytest.mark.parametrize(
    ("base", "build_base"),
    [
        ("core24", None),
        ("core24", "core20"),
        ("core24", "core22"),
        ("core24", "core24"),
        ("core24", "devel"),
    ],
)
def test_application_dotnet_not_registered(base, build_base, snapcraft_yaml):
    """dotnet plugin is disabled for core24 and newer bases."""
    snapcraft_yaml(base=base, build_base=build_base)
    app = application.create_app()

    app._register_default_plugins()

    assert "dotnet" not in craft_parts.plugins.get_registered_plugins()


def test_default_command_integrated(monkeypatch, mocker, new_dir):
    """Test that for core24 projects we accept "pack" as the default command."""

    # Pretend this is an Ubuntu 24.04 system, to match the project's build-base
    mocker.patch.object(
        util, "get_host_base", return_value=bases.BaseName("ubuntu", "24.04")
    )

    snap_dir = new_dir / "snap"
    snap_dir.mkdir()

    # The project itself doesn't really matter.
    project_yaml = snap_dir / "snapcraft.yaml"
    project_yaml.write_text(PARSE_INFO_PROJECT)

    mocked_pack_run = mocker.patch.object(PackCommand, "run", return_value=0)

    monkeypatch.setattr("sys.argv", ["snapcraft", "--destructive-mode"])
    app = application.create_app()
    app.run()

    assert mocked_pack_run.called


@pytest.mark.parametrize("base", const.ESM_BASES)
def test_esm_error(snapcraft_yaml, base):
    """Test that an error is raised when using an ESM base."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)

    app = application.create_app()

    with pytest.raises(
        RuntimeError, match=f"ERROR: base {base!r} was last supported on Snapcraft"
    ):
        app.run()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
def test_esm_pass(mocker, snapcraft_yaml, base):
    """Test that no error is raised when using current supported bases."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_dispatch = mocker.patch(
        "craft_application.application.Application._get_dispatcher"
    )

    app = application.create_app()

    try:
        app.run()
    except ClassicFallback:
        pass
    else:
        mock_dispatch.assert_called_once()


@pytest.mark.parametrize(
    "envvar", ["force-fallback", "disable-fallback", "badvalue", None]
)
@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar(
    monkeypatch,
    snapcraft_yaml,
    base,
    envvar,
    mock_remote_build_run,
    mock_run_legacy,
):
    """Bases core24 and later run new remote-build regardless of envvar."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    if envvar:
        monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", envvar)
    else:
        monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    application.main()

    mock_remote_build_run.assert_called_once()
    mock_run_legacy.assert_not_called()


@pytest.mark.parametrize("base", const.LEGACY_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar_disable_fallback_core20(
    snapcraft_yaml, base, mock_remote_build_run, mock_run_legacy, monkeypatch
):
    """core20 base run new remote-build if envvar is `disable-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "disable-fallback")
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)

    application.main()

    mock_remote_build_run.assert_called_once()
    mock_run_legacy.assert_not_called()


@pytest.mark.parametrize("base", const.LEGACY_BASES | {"core22"})
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar_force_fallback_core22(
    snapcraft_yaml, base, mock_remote_build_run, mock_run_legacy, monkeypatch
):
    """core22 and older bases run legacy remote-build if envvar is `force-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "force-fallback")

    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_run_legacy.assert_called_once()
    mock_remote_build_run.assert_not_called()


@pytest.mark.parametrize("base", const.LEGACY_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar_force_fallback_unset_core20(
    snapcraft_yaml, base, mock_remote_build_run, mock_run_legacy, monkeypatch
):
    """core20 base run legacy remote-build if envvar is unset."""
    monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_run_legacy.assert_called_once()
    mock_remote_build_run.assert_not_called()


@pytest.mark.parametrize("base", {"core22"})
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar_force_fallback_empty_core22(
    snapcraft_yaml, base, mock_remote_build_run, mock_run_legacy, monkeypatch
):
    """core22 bases run craft-application remote-build if envvar is empty."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "")

    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_remote_build_run.assert_called_once()
    mock_run_legacy.assert_not_called()


@pytest.mark.parametrize("base", const.LEGACY_BASES | {"core22"})
@pytest.mark.usefixtures("mock_confirm", "mock_remote_build_argv")
def test_run_envvar_invalid(snapcraft_yaml, base, monkeypatch):
    """core20 and core22 bases raise an error if the envvar is invalid."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "badvalue")

    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    with pytest.raises(SnapcraftError) as err:
        application.main()

    assert err.match(
        "Unknown value 'badvalue' in environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY'. Valid values are 'disable-fallback' and "
        "'force-fallback'"
    )


@pytest.mark.parametrize(
    ("base", "build_base", "is_known_core24"),
    [
        ("core20", None, False),
        ("core20", "core20", False),
        ("core20", "devel", False),
        ("core22", None, False),
        ("core22", "core22", False),
        ("core22", "devel", False),
        ("core24", "core22", True),
        ("core24", None, True),
        ("core24", "core24", True),
        ("core24", "devel", True),
    ],
)
def test_known_core24(snapcraft_yaml, base, build_base, is_known_core24):
    snapcraft_yaml(base=base, build_base=build_base)

    app = application.create_app()

    assert app._known_core24 == is_known_core24


@pytest.mark.parametrize(
    ("message", "resolution", "expected_message"),
    [
        (
            "error message",
            "error resolution",
            "error message\nRecommended resolution: error resolution",
        ),
        ("error message", None, "error message"),
    ],
)
def test_store_error(mocker, capsys, message, resolution, expected_message):
    mocker.patch(
        "snapcraft.application.Application.run",
        side_effect=craft_store.errors.CraftStoreError(message, resolution=resolution),
    )

    return_code = application.main()

    assert return_code == 1
    _, err = capsys.readouterr()
    assert f"craft-store error: {expected_message}" in err


def test_store_key_error(mocker, capsys):
    mocker.patch(
        "snapcraft.application.Application.run",
        side_effect=craft_store.errors.NoKeyringError(),
    )

    return_code = application.main()

    assert return_code == 1
    _, err = capsys.readouterr()
    assert err.startswith(
        # There is merit in showing the line as it would be printed out.
        # If it is too long here it needs fixing at the source.
        # pylint: disable=[line-too-long]
        dedent(
            """\
            craft-store error: No keyring found to store or retrieve credentials from.
            Recommended resolution: Ensure the keyring is working or SNAPCRAFT_STORE_CREDENTIALS is correctly exported into the environment
            For more information, check out: https://snapcraft.io/docs/snapcraft-authentication
        """
            # pylint: enable=[line-too-long]
        )
    )
