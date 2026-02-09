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
import re
import sys
from textwrap import dedent
from typing import cast

import craft_application.launchpad
import craft_application.remote
import craft_cli
import craft_parts.plugins
import craft_parts.plugins.dotnet_plugin
import craft_parts.plugins.dotnet_v2_plugin
import craft_store
import pytest
import yaml
from craft_application import util
from craft_application.commands import get_other_command_group
from craft_parts.packages import snaps
from craft_platforms import DebianArchitecture
from craft_providers import bases

from snapcraft import application, cli, const, services
from snapcraft.commands import PackCommand
from snapcraft.errors import ClassicFallback
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
def mock_remote_build_run(mocker):
    _mock_remote_build_run = mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run"
    )
    return _mock_remote_build_run


@pytest.fixture()
def mock_remote_build_argv(mocker):
    """Mock `snapcraft remote-build` cli."""
    return mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-accept-public-upload"]
    )


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

    services.register_snapcraft_services()
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
    extension_source["build-packages"] = [{"to s390x": "test-package"}]
    extension_source["build-snaps"] = [{"to s390x": "test-snap"}]
    extension_source["platforms"] = {
        "s390x": {
            "build-on": str(DebianArchitecture.from_host()),
            "build-for": "s390x",
        },
    }

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

    project = app.services.get("project").get()
    assert "fake-extension/fake-part" in project.parts
    assert project.parts["snapcraft/core"]["build-packages"] == ["test-package"]
    assert project.parts["snapcraft/core"]["build-snaps"] == ["test-snap"]


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


def test_get_project_parse_info(in_project_path):
    """Test that parse-info data is correctly extracted and stored when loading
    the project from a YAML file."""
    snap_dir = in_project_path / "snap"
    snap_dir.mkdir()
    project_yaml = snap_dir / "snapcraft.yaml"
    project_yaml.write_text(PARSE_INFO_PROJECT)

    app = application.create_app()
    app.services.update_kwargs("project", project_dir=in_project_path)
    project_service = cast(services.Project, app.services.get("project"))
    parse_info = project_service.get_parse_info()

    assert parse_info == {"parse-info-part": ["usr/share/metainfo/app.metainfo.xml"]}


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
    ("base", "build_base", "expected_plugin"),
    [
        ("core22", None, craft_parts.plugins.dotnet_plugin.DotnetPlugin),
        ("core22", "core22", craft_parts.plugins.dotnet_plugin.DotnetPlugin),
        ("core22", "devel", craft_parts.plugins.dotnet_plugin.DotnetPlugin),
        ("core24", None, craft_parts.plugins.dotnet_v2_plugin.DotnetV2Plugin),
        ("core24", "core22", craft_parts.plugins.dotnet_v2_plugin.DotnetV2Plugin),
        ("core24", "core24", craft_parts.plugins.dotnet_v2_plugin.DotnetV2Plugin),
        ("core24", "devel", craft_parts.plugins.dotnet_v2_plugin.DotnetV2Plugin),
    ],
)
def test_application_dotnet_registered(
    base, build_base, expected_plugin, snapcraft_yaml
):
    """dotnet plugin is enabled for core22 and later."""
    snapcraft_yaml(base=base, build_base=build_base)
    app = application.create_app()

    app._register_default_plugins()

    assert "dotnet" in craft_parts.plugins.get_registered_plugins()
    assert craft_parts.plugins.get_plugin_class("dotnet") == expected_plugin


def test_application_maven_use_not_registered(snapcraft_yaml):
    """maven-use plugin is disabled."""
    snapcraft_yaml(base="core24")
    app = application.create_app()

    app._register_default_plugins()

    assert "maven-use" not in craft_parts.plugins.get_registered_plugins()


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
def test_esm_error(snapcraft_yaml, base, monkeypatch, capsys):
    """Test that an error is raised when using an ESM base."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    monkeypatch.setattr("sys.argv", ["snapcraft", "pack"])

    application.main()

    _, err = capsys.readouterr()

    assert re.match(
        rf"^Base {base!r} is not supported by this version of Snapcraft.\n"
        rf"Recommended resolution: Use Snapcraft .* from the '.*' channel of snapcraft where {base!r} was last supported.\n"
        r"For more information, check out: .*/reference/bases\n",
        err,
    )


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


def test_yaml_syntax_error(in_project_path, monkeypatch, capsys):
    """Provide a user friendly error on yaml syntax errors."""
    (in_project_path / "snapcraft.yaml").write_text("bad:\nyaml")
    monkeypatch.setattr("sys.argv", ["snapcraft", "pack"])

    application.main()

    _, err = capsys.readouterr()
    assert re.match(
        "^error parsing 'snapcraft\\.yaml': .*\nDetailed information:",
        err,
    )


@pytest.mark.parametrize(
    "bad_yaml",
    [
        """
            name: test
            base: core24
            confinement: strict

            apps:
              # items under `test-app` are missing leading dashes
              test-app:
                foo
                bar
              plugs:
                - home
                - network
        """,
        """
            name: test
            base: core24
            confinement: strict

            apps:
                # `plugs` should be indented under `test-app`
                test-app:
                  command: test-part
                plugs:
                  - home
                  - network
        """,
    ],
)
@pytest.mark.usefixtures("emitter")
def test_yaml_indentation_error(bad_yaml, in_project_path, monkeypatch, capsys):
    """Provide a user friendly error message on schema errors that are not YAML syntax errors."""
    (in_project_path / "snapcraft.yaml").write_text(dedent(bad_yaml))
    monkeypatch.setattr("sys.argv", ["snapcraft", "pack"])

    application.main()

    _, err = capsys.readouterr()
    assert "Bad snapcraft.yaml content:" in err


@pytest.mark.parametrize("command", ["plugins", "list-plugins"])
@pytest.mark.parametrize(
    "args",
    [
        [],
        ["--base", "core22"],
        ["--base=core22"],
        ["--base", "core24"],
        ["--base=core24"],
    ],
)
@pytest.mark.parametrize("base", const.CURRENT_BASES | {None})
def test_run_list_plugins(command, args, base, mocker, monkeypatch, snapcraft_yaml):
    """Do not trigger a classic fallback for `list-plugins` for core22 or newer."""
    monkeypatch.setattr("sys.argv", ["snapcraft", command, *args])
    if base:
        snapcraft_yaml(base=base)
    mock_dispatch = mocker.patch(
        "craft_application.application.Application._get_dispatcher"
    )

    app = application.create_app()
    app.run()

    mock_dispatch.assert_called_once()


@pytest.mark.parametrize("base", const.CURRENT_BASES | {None})
def test_run_version(base, mocker, monkeypatch, snapcraft_yaml):
    """Do not trigger a classic fallback for `version`."""
    monkeypatch.setattr("sys.argv", ["snapcraft", "version"])
    if base:
        snapcraft_yaml(base=base)
    mock_dispatch = mocker.patch(
        "craft_application.application.Application._get_dispatcher"
    )

    app = application.create_app()
    app.run()

    mock_dispatch.assert_called_once()


@pytest.mark.parametrize(
    ("base", "build_base", "use_craftapp_lib"),
    [
        ("core22", None, False),
        ("core22", "core22", False),
        ("core22", "devel", False),
        ("core24", "core22", True),
        ("core24", None, True),
        ("core24", "core24", True),
        ("core24", "devel", True),
        ("core26", None, True),
        ("core26", "core26", True),
        ("core26", "devel", True),
    ],
)
def test_use_craftapp_lib(snapcraft_yaml, base, build_base, use_craftapp_lib):
    snapcraft_yaml(base=base, build_base=build_base)

    app = application.create_app()

    assert app._use_craftapp_lib == use_craftapp_lib


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
        "snapcraft.application.Application._run_inner",
        side_effect=craft_store.errors.CraftStoreError(message, resolution=resolution),
    )

    return_code = application.main()

    assert return_code == 1
    _, err = capsys.readouterr()
    assert expected_message in err


def test_store_key_error(mocker, capsys):
    mocker.patch(
        "snapcraft.application.Application._run_inner",
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
            No keyring found to store or retrieve credentials from.
            Recommended resolution: Ensure the keyring is working or SNAPCRAFT_STORE_CREDENTIALS is correctly exported into the environment
            For more information, check out: https://documentation.ubuntu.com/snapcraft/stable/how-to/publishing/authenticate
        """
            # pylint: enable=[line-too-long]
        )
    )


@pytest.mark.parametrize(
    "error_class",
    [
        craft_application.remote.RemoteBuildError,
        craft_application.launchpad.LaunchpadError,
    ],
)
def test_remote_build_error(mocker, capsys, error_class):
    """Catch remote build errors and include a documentation link."""
    mocker.patch(
        "snapcraft.application.Application._run_inner",
        side_effect=error_class("test-error"),
    )

    return_code = application.main()

    assert return_code == 1
    _, err = capsys.readouterr()

    assert re.match(
        r"^test-error\n"
        r"For more information, check out: .*/explanation/remote-build\n",
        err,
    )


@pytest.mark.parametrize(
    "command",
    {
        command.name
        for group in [
            *cli.COMMAND_GROUPS,
            cli.CORE22_LIFECYCLE_COMMAND_GROUP,
            cli.CORE24_LIFECYCLE_COMMAND_GROUP,
            get_other_command_group(),
        ]
        for command in group.commands
    },
)
def test_get_argv_command(command, monkeypatch):
    """Get the command."""
    monkeypatch.setattr(
        "sys.argv",
        [
            "snapcraft",
            "--verbositytrace",
            "--build-for=armhf",
            "--shell-after",
            command,
        ],
    )

    app = application.create_app()
    actual_command = app._get_argv_command()

    assert actual_command == command


@pytest.mark.parametrize(
    ["args", "expected_command"],
    [
        # no default command
        ([], None),
        (["--verbosity=trace"], None),
        (["--verbosity", "trace"], None),
        (["--shell-after"], None),
        # no options
        (["pack"], "pack"),
        # with an option
        (["pack", "--verbosity=trace"], "pack"),
        (["pack", "--verbosity", "trace"], "pack"),
        (["--verbosity=trace", "pack"], "pack"),
        (["--verbosity", "trace", "pack"], "pack"),
        # with a flag
        (["pack", "--shell-after"], "pack"),
        (["--shell-after", "pack"], "pack"),
        # with an option and a flag
        (["pack", "--verbosity=trace", "--shell-after"], "pack"),
        (["--shell-after", "pack", "--verbosity=trace", "--shell-after"], "pack"),
        (["pack", "--verbosity", "trace", "--shell-after"], "pack"),
        (["--shell-after", "pack", "--verbosity", "trace"], "pack"),
        (["--verbosity=trace", "pack", "--shell-after"], "pack"),
        (["--shell-after", "--verbosity=trace", "pack"], "pack"),
        (["--verbosity", "trace", "pack"], "pack"),
        (["--shell-after", "--verbosity", "trace", "pack"], "pack"),
    ],
)
def test_get_argv_command_with_options(args, expected_command, monkeypatch):
    """Get the command no with a variety of options."""
    monkeypatch.setattr("sys.argv", ["snapcraft", *args])

    app = application.create_app()
    command = app._get_argv_command()

    assert command == expected_command
