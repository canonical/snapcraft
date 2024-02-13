# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

import shlex
import sys
from pathlib import Path
from subprocess import CalledProcessError
from textwrap import dedent
from unittest.mock import Mock, call

import pytest
from craft_providers.bases import BuilddBaseAlias
from craft_providers.multipass import MultipassProvider

from snapcraft import cli, models
from snapcraft.commands.lint import LintCommand
from snapcraft.errors import SnapcraftError
from snapcraft.meta.snap_yaml import SnapMetadata


@pytest.fixture
def fake_assert_file(tmp_path):
    """Returns a path to a fake assertion file."""
    return tmp_path / "test-snap.assert"


@pytest.fixture
def fake_snap_file(tmp_path):
    """Return a path to a fake snap file."""
    return tmp_path / "test-snap.snap"


@pytest.fixture
def fake_snap_metadata():
    data = {
        "name": "test",
        "version": "1.0",
        "summary": "test",
        "description": "test",
        "confinement": "strict",
        "grade": "stable",
        "architectures": ["test"],
    }
    return SnapMetadata.unmarshal(data)


@pytest.fixture
def fake_snapcraft_project():
    data = {
        "name": "test-name",
        "base": "core22",
        "grade": "stable",
        "confinement": "strict",
        "description": "test description",
        "version": "1.0",
        "summary": "test summary",
        "parts": {"part1": {"plugin": "nil"}},
    }
    return models.Project.unmarshal(data)


@pytest.fixture
def mock_argv(mocker, fake_snap_file):
    """Mock `snapcraft lint` cli for a snap named `test-snap.snap`."""
    return mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(fake_snap_file)])


@pytest.fixture
def mock_capture_logs_from_instance(mocker):
    return mocker.patch("snapcraft.commands.lint.providers.capture_logs_from_instance")


@pytest.fixture
def mock_ensure_provider_is_available(mocker):
    return mocker.patch(
        "snapcraft.parts.lifecycle.providers.ensure_provider_is_available"
    )


@pytest.fixture
def mock_get_base_configuration(mocker):
    return mocker.patch("snapcraft.parts.lifecycle.providers.get_base_configuration")


@pytest.fixture
def mock_is_managed_mode(mocker):
    return mocker.patch("snapcraft.commands.lint.is_managed_mode", return_value=False)


@pytest.fixture
def mock_provider(mocker, mock_instance, fake_provider):
    _mock_provider = Mock(wraps=fake_provider)
    mocker.patch(
        "snapcraft.commands.lint.providers.get_provider", return_value=_mock_provider
    )
    return _mock_provider


@pytest.fixture
def mock_run_linters(mocker):
    return mocker.patch(
        "snapcraft.commands.lint.linters.run_linters", return_value=Mock()
    )


@pytest.fixture
def mock_report(mocker):
    return mocker.patch("snapcraft.commands.lint.linters.report")


def test_lint_default(
    emitter,
    fake_assert_file,
    fake_snap_file,
    mock_argv,
    mock_capture_logs_from_instance,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
):
    """Test the lint command prepares an instance."""
    # create a snap file and assertion file
    fake_snap_file.touch()
    fake_assert_file.touch()

    cli.run()

    mock_ensure_provider_is_available.assert_called_once()
    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        http_proxy=None,
        https_proxy=None,
        instance_name="snapcraft-linter",
    )
    assert mock_instance.push_file.mock_calls == [
        call(source=fake_snap_file, destination=Path("/root/test-snap.snap")),
        call(source=fake_assert_file, destination=Path("/root/test-snap.assert")),
    ]
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    mock_capture_logs_from_instance.assert_called_once()
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call("debug", f"Found assertion file {str(fake_assert_file)!r}."),
            call("progress", "Checking build provider availability."),
            call("progress", "Launching instance."),
            call("debug", "running 'snapcraft lint /root/test-snap.snap' in instance"),
        ]
    )


def test_lint_http_https_proxy(
    emitter,
    fake_snap_file,
    mock_capture_logs_from_instance,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
):
    """Pass the http and https proxies into the instance."""
    # create a snap file
    fake_snap_file.touch()

    # mock command
    mocker.patch.object(
        sys,
        "argv",
        [
            "snapcraft",
            "lint",
            str(fake_snap_file),
            "--http-proxy",
            "test-http-proxy",
            "--https-proxy",
            "test-https-proxy",
        ],
    )

    cli.run()

    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        http_proxy="test-http-proxy",
        https_proxy="test-https-proxy",
        instance_name="snapcraft-linter",
    )


def test_lint_assert_file_missing(
    emitter,
    fake_assert_file,
    fake_snap_file,
    mock_argv,
    mock_capture_logs_from_instance,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
):
    """Do not push non-existent assertion files in the instance."""
    # create a snap file
    fake_snap_file.touch()

    cli.run()

    mock_instance.push_file.assert_called_once_with(
        source=fake_snap_file,
        destination=Path("/root/test-snap.snap"),
    )
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    emitter.assert_debug(f"Assertion file {str(fake_assert_file)!r} does not exist.")


def test_lint_assert_file_not_valid(
    emitter,
    fake_assert_file,
    fake_snap_file,
    mock_argv,
    mock_capture_logs_from_instance,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
):
    """Do not push invalid assertion files in the instance."""
    # create a snap file
    fake_snap_file.touch()

    # make the assertion filepath a directory
    fake_assert_file.mkdir()

    cli.run()

    mock_instance.push_file.assert_called_once_with(
        source=fake_snap_file,
        destination=Path("/root/test-snap.snap"),
    )
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    emitter.assert_debug(
        f"Assertion file {str(fake_assert_file)!r} is not a valid file."
    )


def test_lint_multipass_not_supported(
    capsys,
    fake_provider,
    fake_snap_file,
    mock_argv,
    mock_ensure_provider_is_available,
    mock_is_managed_mode,
    mocker,
):
    """Raise an error if Multipass is used as the build provider."""
    _mock_provider = Mock(wraps=fake_provider, spec=MultipassProvider)
    mocker.patch(
        "snapcraft.commands.lint.providers.get_provider", return_value=_mock_provider
    )

    # create a snap file
    fake_snap_file.touch()

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert (
        "'snapcraft lint' is not supported with Multipass as the build provider" in err
    )


def test_lint_default_snap_file_missing(capsys, fake_snap_file, mock_argv):
    """Raise an error if the snap file does not exist."""
    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"snap file {str(fake_snap_file)!r} does not exist" in err


def test_lint_default_snap_file_not_valid(capsys, fake_snap_file, mock_argv):
    """Raise an error if the snap file is not valid."""
    # make the snap filepath a directory
    fake_snap_file.mkdir()

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"snap file {str(fake_snap_file)!r} is not a valid file" in err


def test_lint_execute_run_error(
    capsys,
    emitter,
    fake_snap_file,
    mock_argv,
    mock_capture_logs_from_instance,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
):
    """Raise an error if running snapcraft in the instance fails."""
    # create a snap file
    fake_snap_file.touch()

    mock_instance.execute_run.side_effect = CalledProcessError(cmd="err", returncode=1)

    cli.run()

    mock_capture_logs_from_instance.assert_called_once()
    out, err = capsys.readouterr()
    assert not out
    assert "failed to execute 'snapcraft lint /root/test-snap.snap' in instance" in err


@pytest.mark.parametrize("confinement", ["classic", "strict"])
@pytest.mark.parametrize("grade", ["devel", "stable"])
def test_lint_managed_mode(
    confinement,
    emitter,
    fake_assert_file,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    grade,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """Run the linter in managed mode."""
    mock_is_managed_mode.return_value = True

    # create a snap file
    fake_snap_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)]
    )

    # build snap install command
    command = ["snap", "install", str(fake_snap_file)]
    if confinement == "classic":
        command.append("--classic")
    command.append("--dangerous")
    if grade == "devel":
        command.append("--devmode")
    fake_process.register_subprocess(command)

    # mock data from the unsquashed snap
    fake_snap_metadata.confinement = confinement
    fake_snap_metadata.grade = grade
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    mock_run_linters.assert_called_once_with(
        lint=models.Lint(ignore=["classic"]),
        location=Path("/snap/test/current"),
    )
    mock_report.assert_called_once_with(
        mock_run_linters.return_value, intermediate=True
    )
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call("debug", f"Assertion file {str(fake_assert_file)!r} does not exist."),
            call("progress", f"Unsquashing snap file {fake_snap_file.name!r}."),
            call("progress", f"Installing snap with {shlex.join(command)!r}."),
            call("verbose", "No lint filters defined in 'snapcraft.yaml'."),
        ]
    )


def test_lint_managed_mode_without_snapcraft_yaml(
    emitter,
    fake_assert_file,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """Run the linter in managed mode without a snapcraft.yaml file."""
    mock_is_managed_mode.return_value = True

    # create a snap file
    fake_snap_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)]
    )
    fake_process.register_subprocess(
        ["snap", "install", str(fake_snap_file), "--dangerous"]
    )

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=None,
    )

    cli.run()

    mock_run_linters.assert_called_once_with(
        lint=models.Lint(ignore=["classic"]),
        location=Path("/snap/test/current"),
    )
    mock_report.assert_called_once_with(
        mock_run_linters.return_value, intermediate=True
    )
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call("debug", f"Assertion file {str(fake_assert_file)!r} does not exist."),
            call("progress", f"Unsquashing snap file {fake_snap_file.name!r}."),
            call(
                "progress",
                f"Installing snap with 'snap install {str(fake_snap_file)} "
                "--dangerous'.",
            ),
            call(
                "verbose",
                "Not loading lint filters from 'snapcraft.yaml' because the file does "
                "not exist inside the snap file.",
            ),
            call(
                "verbose",
                "To include 'snapcraft.yaml' in a snap file, use the parameter "
                "'--enable-manifest' when building the snap.",
            ),
        ]
    )


def test_lint_managed_mode_unsquash_error(
    capsys,
    emitter,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """Raise an error if the snap file cannot be installed."""
    mock_is_managed_mode.return_value = True

    # create a snap file
    fake_snap_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)],
        returncode=1,
    )

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"could not unsquash snap file {fake_snap_file.name!r}" in err


def test_lint_managed_mode_snap_install_error(
    capsys,
    emitter,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """Raise an error if the snap file cannot be installed."""
    mock_is_managed_mode.return_value = True

    # create a snap file
    fake_snap_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)]
    )
    fake_process.register_subprocess(
        ["snap", "install", str(fake_snap_file), "--dangerous"], returncode=1
    )

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"could not install snap file {fake_snap_file.name!r}" in err


def test_lint_managed_mode_assert(
    emitter,
    fake_assert_file,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """Run the linter in managed mode with an assert file."""
    mock_is_managed_mode.return_value = True

    # create a snap file and assertion file
    fake_snap_file.touch()
    fake_assert_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)]
    )
    fake_process.register_subprocess(["snap", "ack", str(fake_assert_file)])
    fake_process.register_subprocess(["snap", "install", str(fake_snap_file)])

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    mock_run_linters.assert_called_once_with(
        lint=models.Lint(ignore=["classic"]),
        location=Path("/snap/test/current"),
    )
    mock_report.assert_called_once_with(
        mock_run_linters.return_value, intermediate=True
    )
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call("debug", f"Found assertion file {str(fake_assert_file)!r}."),
            call("progress", "Unsquashing snap file 'test-snap.snap'."),
            call(
                "progress",
                f"Installing assertion file with 'snap ack {fake_assert_file}'.",
            ),
            call("progress", f"Installing snap with 'snap install {fake_snap_file}'."),
            call("verbose", "No lint filters defined in 'snapcraft.yaml'."),
        ]
    )


def test_lint_managed_mode_assert_error(
    emitter,
    fake_assert_file,
    fake_process,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
):
    """If the assert file fails to be installed, install the snap dangerously."""
    mock_is_managed_mode.return_value = True

    # create a snap file and assertion file
    fake_snap_file.touch()
    fake_assert_file.touch()

    # register subprocess calls
    fake_process.register_subprocess(
        ["unsquashfs", "-force", "-dest", fake_process.any(), str(fake_snap_file)]
    )
    fake_process.register_subprocess(
        ["snap", "ack", str(fake_assert_file)], returncode=1
    )
    fake_process.register_subprocess(
        ["snap", "install", str(fake_snap_file), "--dangerous"]
    )

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    mock_run_linters.assert_called_once_with(
        lint=models.Lint(ignore=["classic"]),
        location=Path("/snap/test/current"),
    )
    mock_report.assert_called_once_with(
        mock_run_linters.return_value, intermediate=True
    )
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call("debug", f"Found assertion file {str(fake_assert_file)!r}."),
            call("progress", "Unsquashing snap file 'test-snap.snap'."),
            call(
                "progress",
                f"Installing assertion file with 'snap ack {fake_assert_file}'.",
            ),
            call(
                "progress",
                "Could not add assertions from file 'test-snap.assert'",
                permanent=True,
            ),
            call(
                "progress",
                f"Installing snap with 'snap install {fake_snap_file} --dangerous'.",
            ),
            call("verbose", "No lint filters defined in 'snapcraft.yaml'."),
        ]
    )


@pytest.mark.parametrize(
    ["project_lint", "expected_lint"],
    [
        (
            models.Lint(ignore=[]),
            models.Lint(ignore=["classic"]),
        ),
        (
            models.Lint(ignore=["library"]),
            models.Lint(ignore=["library", "classic"]),
        ),
        (
            models.Lint(ignore=["library", "classic"]),
            models.Lint(ignore=["library", "classic"]),
        ),
        (
            models.Lint(ignore=[{"classic": ["bin/test1", "bin/test2"]}]),
            models.Lint(ignore=["classic"]),
        ),
        (
            models.Lint(ignore=["library", {"classic": ["bin/test1", "bin/test2"]}]),
            models.Lint(ignore=["library", "classic"]),
        ),
        (
            models.Lint(
                ignore=["library", "classic", {"classic": ["bin/test1", "bin/test2"]}]
            ),
            models.Lint(ignore=["library", "classic"]),
        ),
    ],
)
def test_lint_managed_mode_with_lint_config(
    emitter,
    expected_lint,
    fake_snap_file,
    fake_snap_metadata,
    fake_snapcraft_project,
    mock_argv,
    mock_is_managed_mode,
    mock_report,
    mock_run_linters,
    mocker,
    project_lint,
):
    """Run the linter in managed mode and process the lint config from the project."""
    mock_is_managed_mode.return_value = True
    mocker.patch("subprocess.run")

    # create a snap file
    fake_snap_file.touch()

    # mock data from the unsquashed snap
    mocker.patch(
        "snapcraft.commands.lint.snap_yaml.read", return_value=fake_snap_metadata
    )

    # add a lint config to the project
    fake_snapcraft_project.lint = project_lint
    mocker.patch(
        "snapcraft.commands.lint.LintCommand._load_project",
        return_value=fake_snapcraft_project,
    )

    cli.run()

    # lint config from project should be passed to `run_linter()`
    mock_run_linters.assert_called_once_with(
        lint=expected_lint, location=Path("/snap/test/current")
    )
    mock_report.assert_called_once_with(
        mock_run_linters.return_value, intermediate=True
    )
    emitter.assert_verbose("Collected lint config from 'snapcraft.yaml'.")


def test_load_project(fake_snapcraft_project, tmp_path):
    """Load a simple snapcraft.yaml project.

    To simplify the unit tests, the `_load_project()` method is mocked out of the other
    tests and tested separately.
    """
    # create a simple snapcraft.yaml
    (tmp_path / "snap").mkdir()
    snap_file = tmp_path / "snap/snapcraft.yaml"
    with snap_file.open("w") as yaml_file:
        print(
            dedent(
                """\
                name: test-name
                version: "1.0"
                summary: test summary
                description: test description
                base: core22
                confinement: strict
                grade: stable

                parts:
                  part1:
                    plugin: nil
                """
            ),
            file=yaml_file,
        )

    result = LintCommand(None)._load_project(snapcraft_yaml_file=snap_file)

    assert result == fake_snapcraft_project


@pytest.mark.usefixtures("fake_extension")
def test_load_project_complex(mocker, tmp_path):
    """Load a complex snapcraft file.

    This includes lint, parse-info, architectures, and advanced grammar.
    """
    # mock for advanced grammar parsing (i.e. `on amd64:`)
    mocker.patch("snapcraft.commands.lint.get_host_architecture", return_value="amd64")

    # create a snap file
    (tmp_path / "snap").mkdir()
    snap_file = tmp_path / "snap/snapcraft.yaml"
    with snap_file.open("w") as yaml_file:
        print(
            dedent(
                """\
                name: test-name
                version: "1.0"
                summary: test summary
                description: test description
                base: core22
                confinement: strict
                grade: stable
                architectures: [amd64, arm64, armhf]

                apps:
                  app1:
                    command: app1
                    command-chain: [fake-command]
                    extensions: [fake-extension]

                parts:
                  nil:
                    plugin: nil
                    parse-info:
                      - usr/share/metainfo/app1.appdata.xml
                    stage-packages:
                      - mesa-opencl-icd
                      - ocl-icd-libopencl1
                      - on amd64:
                        - intel-opencl-icd
                """
            ),
            file=yaml_file,
        )

    result = LintCommand(None)._load_project(snapcraft_yaml_file=snap_file)
    assert result == models.Project.unmarshal(
        {
            "name": "test-name",
            "base": "core22",
            "build_base": "core22",
            "version": "1.0",
            "summary": "test summary",
            "description": "test description",
            "confinement": "strict",
            "grade": "stable",
            "architectures": [{"build_on": ["amd64"], "build_for": ["amd64"]}],
            "apps": {
                "app1": {
                    "command": "app1",
                    "plugs": ["fake-plug"],
                    "command-chain": ["fake-command"],
                }
            },
            "parts": {
                "nil": {
                    "plugin": "nil",
                    "stage-packages": [
                        "mesa-opencl-icd",
                        "ocl-icd-libopencl1",
                        "intel-opencl-icd",
                    ],
                    "after": ["fake-extension/fake-part"],
                },
                "fake-extension/fake-part": {"plugin": "nil"},
            },
        }
    )


def test_load_project_no_file(emitter, tmp_path):
    """Return None if there is no snapcraft.yaml file."""
    snapcraft_yaml_file = tmp_path / "snap/snapcraft.yaml"

    result = LintCommand(None)._load_project(snapcraft_yaml_file=snapcraft_yaml_file)

    assert not result
    emitter.assert_debug(f"Could not find {snapcraft_yaml_file.name!r}.")


@pytest.mark.parametrize("base", ["core", "core18", "core20"])
def test_load_project_unsupported_core_error(base, tmp_path):
    """Raise an error if for snaps with core, core18, and core20 bases."""
    # create a simple snapcraft.yaml
    (tmp_path / "snap").mkdir()
    snap_file = tmp_path / "snap/snapcraft.yaml"
    snap_file.write_text(
        dedent(
            f"""\
            name: test-name
            version: "1.0"
            summary: test summary
            description: test description
            base: {base}
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
            """
        )
    )

    with pytest.raises(SnapcraftError) as raised:
        LintCommand(None)._load_project(snapcraft_yaml_file=snap_file)

    assert str(raised.value) == "can not lint snap using a base older than core22"
