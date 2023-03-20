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

import argparse
import shutil
import textwrap
from datetime import datetime
from pathlib import Path
from unittest.mock import ANY, Mock, PropertyMock, call

import pytest
from craft_cli import EmitterMode, emit
from craft_parts import Action, Step, callbacks
from craft_providers.bases.buildd import BuilddBaseAlias

from snapcraft import errors
from snapcraft.elf import ElfFile
from snapcraft.parts import lifecycle as parts_lifecycle
from snapcraft.parts.update_metadata import update_project_metadata
from snapcraft.projects import MANDATORY_ADOPTABLE_FIELDS, Architecture, Project
from snapcraft.utils import get_host_architecture

_SNAPCRAFT_YAML_FILENAMES = [
    "snap/snapcraft.yaml",
    "build-aux/snap/snapcraft.yaml",
    "snapcraft.yaml",
    ".snapcraft.yaml",
]


@pytest.fixture(autouse=True)
def disable_install(mocker):
    mocker.patch("craft_parts.packages.Repository.install_packages")
    mocker.patch("craft_parts.packages.snaps.install_snaps")


@pytest.fixture(autouse=True)
def unregister_callbacks(mocker):
    callbacks.unregister_all()


@pytest.fixture(autouse=True)
def disable_getxattrs(mocker):
    mocker.patch("os.getxattr", new=lambda x, y: b"pkg")


@pytest.fixture
def project_vars(mocker):
    yield mocker.patch(
        "snapcraft.parts.PartsLifecycle.project_vars",
        new_callable=PropertyMock,
        return_value={"version": "0.1", "grade": "stable"},
    )


@pytest.fixture()
def mock_provider(mocker, mock_instance, fake_provider):
    _mock_provider = Mock(wraps=fake_provider)
    mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_provider", return_value=_mock_provider
    )
    yield _mock_provider


@pytest.fixture()
def mock_get_instance_name(mocker):
    yield mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_instance_name",
        return_value="test-instance-name",
    )


def test_config_not_found(new_dir):
    """If snapcraft.yaml is not found, raise an error."""
    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run("pull", argparse.Namespace())

    assert str(raised.value) == (
        "Could not find snap/snapcraft.yaml. Are you sure you are in the right "
        "directory?"
    )
    assert raised.value.resolution == "To start a new project, use `snapcraft init`"


def test_config_loading_error(new_dir, mocker, snapcraft_yaml):
    """Catch OSErrors when loading snapcraft.yaml"""
    mocker.patch(
        "builtins.open",
        side_effect=OSError(2, "test-message", "test-filename"),
    )
    snapcraft_yaml(base="core22")
    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run("pull", argparse.Namespace())

    assert str(raised.value) == ("test-message: 'test-filename'.")


@pytest.mark.parametrize("filename", _SNAPCRAFT_YAML_FILENAMES)
def test_snapcraft_yaml_load(new_dir, snapcraft_yaml, filename, mocker):
    """Snapcraft.yaml should be parsed as a valid yaml file."""
    yaml_data = snapcraft_yaml(base="core22", filename=filename)
    run_command_mock = mocker.patch("snapcraft.parts.lifecycle._run_command")
    mocker.patch("snapcraft.utils.get_parallel_build_count", return_value=5)

    parts_lifecycle.run(
        "pull",
        argparse.Namespace(
            parts=["part1"],
            destructive_mode=True,
            use_lxd=False,
            provider=None,
            enable_manifest=False,
            manifest_image_information=None,
            bind_ssh=False,
            ua_token=None,
            build_for=None,
        ),
    )

    project = Project.unmarshal(yaml_data)

    if filename == "build-aux/snap/snapcraft.yaml":
        assets_dir = Path("build-aux/snap")
    else:
        assets_dir = Path("snap")

    assert run_command_mock.mock_calls == [
        call(
            "pull",
            project=project,
            parse_info={},
            assets_dir=assets_dir,
            parallel_build_count=5,
            start_time=mocker.ANY,
            parsed_args=argparse.Namespace(
                parts=["part1"],
                destructive_mode=True,
                use_lxd=False,
                provider=None,
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                ua_token=None,
                build_for=None,
            ),
        ),
    ]


@pytest.mark.parametrize(
    "cmd", ["pull", "build", "stage", "prime", "pack", "snap", "clean"]
)
def test_lifecycle_run_provider(cmd, snapcraft_yaml, new_dir, mocker):
    """Option --provider is not supported in core22."""
    snapcraft_yaml(base="core22")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")

    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run(
            cmd,
            parsed_args=argparse.Namespace(
                destructive_mode=False,
                use_lxd=False,
                provider="some",
                build_for=get_host_architecture(),
            ),
        )

    assert run_mock.mock_calls == []
    assert str(raised.value) == "Option --provider is not supported."


@pytest.mark.parametrize("cmd", ["pull", "build", "stage", "prime", "snap", "clean"])
def test_lifecycle_legacy_run_provider(cmd, snapcraft_yaml, new_dir, mocker):
    """Option --provider is supported by legacy."""
    snapcraft_yaml(base="core20")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")

    with pytest.raises(errors.LegacyFallback) as raised:
        parts_lifecycle.run(
            cmd,
            parsed_args=argparse.Namespace(
                destructive_mode=False,
                use_lxd=False,
                provider="some",
            ),
        )

    assert run_mock.mock_calls == []
    assert str(raised.value) == "base is core20"


@pytest.mark.parametrize(
    "cmd", ["pull", "build", "stage", "prime", "pack", "snap", "clean"]
)
def test_lifecycle_run_ua_services_without_token(cmd, snapcraft_yaml, new_dir, mocker):
    """UA services require --ua-token."""
    snapcraft_yaml(base="core22", **{"ua-services": ["svc1", "svc2"]})
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")

    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run(
            cmd,
            parsed_args=argparse.Namespace(
                destructive_mode=False,
                use_lxd=False,
                provider=None,
                ua_token=None,
                build_for=get_host_architecture(),
            ),
        )

    assert run_mock.mock_calls == []
    assert str(raised.value) == "UA services require a UA token to be specified."


@pytest.mark.parametrize(
    "cmd", ["pull", "build", "stage", "prime", "pack", "snap", "clean"]
)
def test_lifecycle_run_ua_services_without_experimental_flag(
    cmd, snapcraft_yaml, new_dir, mocker
):
    """UA services require --ua-token."""
    snapcraft_yaml(base="core22", **{"ua-services": ["svc1", "svc2"]})
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")

    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run(
            cmd,
            parsed_args=argparse.Namespace(
                destructive_mode=False,
                use_lxd=False,
                provider=None,
                ua_token="my-token",
                build_for=get_host_architecture(),
                enable_experimental_ua_services=False,
            ),
        )

    assert run_mock.mock_calls == []
    assert str(raised.value) == (
        "Using UA services requires --enable-experimental-ua-services."
    )


@pytest.mark.parametrize(
    "cmd,step",
    [
        ("pull", "pull"),
        ("build", "build"),
        ("stage", "stage"),
        ("prime", "prime"),
    ],
)
@pytest.mark.parametrize("debug_shell", [None, "shell", "shell_after"])
def test_lifecycle_run_command_step(
    cmd, step, debug_shell, snapcraft_yaml, project_vars, new_dir, mocker
):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.meta.snap_yaml.write")
    mocker.patch("snapcraft.pack.pack_snap")

    parsed_args = argparse.Namespace(
        debug=False,
        destructive_mode=True,
        enable_manifest=False,
        shell=False,
        shell_after=False,
        use_lxd=False,
        ua_token=None,
        parts=[],
    )

    if debug_shell:
        setattr(parsed_args, debug_shell, True)

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=parsed_args,
    )

    call_args = {"shell": False, "shell_after": False}
    if debug_shell:
        call_args[debug_shell] = True

    assert run_mock.mock_calls == [call(step, **call_args)]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_run_command_pack(cmd, snapcraft_yaml, project_vars, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            enable_manifest=False,
            shell=False,
            shell_after=False,
            use_lxd=False,
            ua_token=None,
            parts=[],
        ),
    )

    assert run_mock.mock_calls == [call("prime", shell=False, shell_after=False)]
    assert pack_mock.mock_calls[:1] == [
        call(
            new_dir / "prime",
            output=None,
            compression="xz",
            name="mytest",
            version="0.1",
            target_arch=get_host_architecture(),
        )
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_destructive_mode(
    cmd, snapcraft_yaml, project_vars, new_dir, mocker
):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    mocker.patch("snapcraft.utils.is_managed_mode", return_value=True)
    mocker.patch(
        "snapcraft.utils.get_managed_environment_home_path",
        return_value=new_dir / "home",
    )

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            enable_manifest=False,
            destructive_mode=True,
            shell=False,
            shell_after=False,
            use_lxd=False,
            ua_token=None,
            parts=[],
        ),
    )

    assert run_in_provider_mock.mock_calls == []
    assert run_mock.mock_calls == [call("prime", shell=False, shell_after=False)]
    assert pack_mock.mock_calls[:1] == [
        call(
            new_dir / "home/prime",
            output=None,
            compression="xz",
            name="mytest",
            version="0.1",
            target_arch=get_host_architecture(),
        )
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_managed(cmd, snapcraft_yaml, project_vars, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    mocker.patch("snapcraft.utils.is_managed_mode", return_value=True)
    mocker.patch(
        "snapcraft.utils.get_managed_environment_home_path",
        return_value=new_dir / "home",
    )

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            bind_ssh=False,
            build_for=None,
            enable_manifest=False,
            manifest_image_information=None,
            destructive_mode=False,
            shell=False,
            shell_after=False,
            use_lxd=False,
            ua_token=None,
            parts=[],
        ),
    )

    assert run_in_provider_mock.mock_calls == []
    assert run_mock.mock_calls == [call("prime", shell=False, shell_after=False)]
    assert pack_mock.mock_calls[:1] == [
        call(
            new_dir / "home/prime",
            output=None,
            compression="xz",
            name="mytest",
            version="0.1",
            target_arch=get_host_architecture(),
        )
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_not_managed(cmd, snapcraft_yaml, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.utils.is_managed_mode", return_value=False)

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=False,
            use_lxd=False,
            parts=[],
        ),
    )

    assert run_mock.mock_calls == []
    assert run_in_provider_mock.mock_calls == [
        call(
            project,
            cmd,
            argparse.Namespace(
                directory=None,
                output=None,
                destructive_mode=False,
                use_lxd=False,
                parts=[],
            ),
        )
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_metadata_error(cmd, snapcraft_yaml, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.utils.is_managed_mode", return_value=True)
    mocker.patch(
        "snapcraft.utils.get_managed_environment_home_path",
        return_value=new_dir / "home",
    )
    mocker.patch(
        "snapcraft.parts.PartsLifecycle.project_vars",
        new_callable=PropertyMock,
        return_value={"version": "0.1", "grade": "invalid"},  # invalid value
    )
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    mocker.patch("snapcraft.meta.snap_yaml.write")

    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle._run_command(
            cmd,
            project=project,
            assets_dir=Path(),
            start_time=datetime.now(),
            parse_info={},
            parallel_build_count=8,
            parsed_args=argparse.Namespace(
                directory=None,
                output=None,
                debug=False,
                destructive_mode=False,
                shell=False,
                shell_after=False,
                use_lxd=False,
                ua_token=None,
                parts=[],
            ),
        )

    assert str(raised.value) == (
        "error setting grade: unexpected value; permitted: 'stable', 'devel'"
    )
    assert run_mock.mock_calls == [call("prime", shell=False, shell_after=False)]
    assert pack_mock.mock_calls == []


@pytest.mark.parametrize("field", MANDATORY_ADOPTABLE_FIELDS)
def test_lifecycle_metadata_empty(field, snapcraft_yaml, new_dir):
    """Adoptable fields shouldn't be empty after adoption."""
    yaml_data = snapcraft_yaml(base="core22")
    yaml_data.pop(field)
    yaml_data["adopt-info"] = "part"
    project = Project.unmarshal(yaml_data)

    with pytest.raises(errors.SnapcraftError) as raised:
        update_project_metadata(
            project,
            project_vars={"version": "", "grade": ""},
            metadata_list=[],
            assets_dir=new_dir,
            prime_dir=new_dir,
        )

    assert str(raised.value) == f"Field {field!r} was not adopted from metadata"


def test_lifecycle_run_command_clean(
    snapcraft_yaml, project_vars, new_dir, mocker, mock_get_instance_name
):
    """Clean provider project when called without parts."""
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    clean_mock = mocker.patch(
        "snapcraft.providers.LXDProvider.clean_project_environments"
    )

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=False,
            use_lxd=False,
            parts=None,
        ),
    )

    assert clean_mock.mock_calls == [call(instance_name="test-instance-name")]


def test_lifecycle_clean_destructive_mode(
    snapcraft_yaml, project_vars, new_dir, mocker
):
    """Clean local project if called in destructive mode."""
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    clean_mock = mocker.patch("snapcraft.parts.PartsLifecycle.clean")

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=True,
            use_lxd=False,
            parts=None,
        ),
    )

    assert clean_mock.mock_calls == [call(part_names=None)]


def test_lifecycle_clean_part_names(snapcraft_yaml, project_vars, new_dir, mocker):
    """Clean project inside provider if called with part names."""
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=False,
            use_lxd=False,
            parts=["part1"],
        ),
    )

    assert run_in_provider_mock.mock_calls == [
        call(
            project,
            "clean",
            argparse.Namespace(
                directory=None,
                output=None,
                destructive_mode=False,
                use_lxd=False,
                parts=["part1"],
            ),
        )
    ]


def test_lifecycle_clean_part_names_destructive_mode(
    snapcraft_yaml, project_vars, new_dir, mocker
):
    """Clean local project if called in destructive mode."""
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    clean_mock = mocker.patch("snapcraft.parts.PartsLifecycle.clean")

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=True,
            use_lxd=False,
            parts=["part1"],
        ),
    )

    assert clean_mock.mock_calls == [call(part_names=["part1"])]


def test_lifecycle_clean_managed(snapcraft_yaml, project_vars, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    clean_mock = mocker.patch("snapcraft.parts.PartsLifecycle.clean")
    mocker.patch("snapcraft.utils.is_managed_mode", return_value=True)
    mocker.patch(
        "snapcraft.utils.get_managed_environment_home_path",
        return_value=new_dir / "home",
    )

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=False,
            use_lxd=False,
            parts=["part1"],
        ),
    )

    assert run_in_provider_mock.mock_calls == []
    assert clean_mock.mock_calls == [call(part_names=["part1"])]


@pytest.mark.parametrize("cmd", ["pull", "build", "stage", "prime", "pack", "snap"])
def test_lifecycle_debug_shell(snapcraft_yaml, cmd, new_dir, mocker):
    """Adoptable fields shouldn't be empty after adoption."""
    mocker.patch("craft_parts.executor.Executor.execute", side_effect=Exception)
    mock_shell = mocker.patch("snapcraft.parts.lifecycle.launch_shell")
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    with pytest.raises(errors.SnapcraftError):
        parts_lifecycle._run_command(
            cmd,
            project=project,
            parse_info={},
            assets_dir=Path(),
            start_time=datetime.now(),
            parallel_build_count=8,
            parsed_args=argparse.Namespace(
                directory=None,
                output=None,
                debug=True,
                destructive_mode=True,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_manifest=False,
                ua_token=None,
                parts=["part1"],
            ),
        )

    assert mock_shell.mock_calls == [call()]


def test_lifecycle_post_lifecycle_debug_shell(snapcraft_yaml, new_dir, mocker):
    """Adoptable fields shouldn't be empty after adoption."""
    mocker.patch("snapcraft.pack.pack_snap", side_effect=Exception)
    mocker.patch("snapcraft.meta.snap_yaml.write")
    mock_shell = mocker.patch("snapcraft.parts.lifecycle.launch_shell")
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    with pytest.raises(errors.SnapcraftError):
        parts_lifecycle._run_command(
            "pack",
            project=project,
            parse_info={},
            assets_dir=Path(),
            start_time=datetime.now(),
            parallel_build_count=8,
            parsed_args=argparse.Namespace(
                directory=None,
                output=None,
                debug=True,
                destructive_mode=True,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_manifest=False,
                parts=["part1"],
            ),
        )

    assert mock_shell.mock_calls == [()]


@pytest.mark.parametrize(
    "cmd,expected_last_step",
    [
        ("pull", None),
        ("build", Step.OVERLAY),
        ("stage", Step.BUILD),
        ("prime", Step.STAGE),
    ],
)
def test_lifecycle_shell(snapcraft_yaml, cmd, expected_last_step, new_dir, mocker):
    """Check if the last step executed before shell is the previous step."""
    last_step = None

    def _fake_execute(_, action: Action, **kwargs):  # pylint: disable=unused-argument
        nonlocal last_step
        last_step = action.step

    mocker.patch("craft_parts.executor.Executor.execute", new=_fake_execute)
    mock_shell = mocker.patch("subprocess.run")
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=True,
            shell_after=False,
            use_lxd=False,
            enable_manifest=False,
            ua_token=None,
            parts=["part1"],
        ),
    )

    assert last_step == expected_last_step
    assert mock_shell.mock_calls == [call(["bash"], check=False, cwd=None)]


@pytest.mark.parametrize(
    "cmd,expected_last_step",
    [
        ("pull", Step.PULL),
        ("build", Step.BUILD),
        ("stage", Step.STAGE),
        ("prime", Step.PRIME),
    ],
)
def test_lifecycle_shell_after(
    snapcraft_yaml, cmd, expected_last_step, new_dir, mocker
):
    """Check if the last step executed before shell is the current step."""
    last_step = None

    def _fake_execute(_, action: Action, **kwargs):  # pylint: disable=unused-argument
        nonlocal last_step
        last_step = action.step

    mocker.patch("craft_parts.executor.Executor.execute", new=_fake_execute)
    mock_shell = mocker.patch("subprocess.run")
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=False,
            shell_after=True,
            use_lxd=False,
            enable_manifest=False,
            ua_token=None,
            parts=["part1"],
        ),
    )

    assert last_step == expected_last_step
    assert mock_shell.mock_calls == [call(["bash"], check=False, cwd=None)]


def test_lifecycle_adopt_project_vars(snapcraft_yaml, new_dir):
    """Adoptable fields shouldn't be empty after adoption."""
    yaml_data = snapcraft_yaml(base="core22")
    yaml_data.pop("version")
    yaml_data.pop("grade")
    yaml_data["adopt-info"] = "part"
    project = Project.unmarshal(yaml_data)

    update_project_metadata(
        project,
        project_vars={"version": "42", "grade": "devel"},
        metadata_list=[],
        assets_dir=new_dir,
        prime_dir=new_dir,
    )

    assert project.version == "42"
    assert project.grade == "devel"


def test_extract_parse_info():
    yaml_data = {
        "name": "foo",
        "parts": {"p1": {"plugin": "nil", "parse-info": "foo/metadata.xml"}, "p2": {}},
    }
    parse_info = parts_lifecycle.extract_parse_info(yaml_data)
    assert yaml_data == {"name": "foo", "parts": {"p1": {"plugin": "nil"}, "p2": {}}}
    assert parse_info == {"p1": "foo/metadata.xml"}


def test_get_snap_project_no_base(snapcraft_yaml, new_dir):
    with pytest.raises(errors.ProjectValidationError) as raised:
        Project.unmarshal(snapcraft_yaml(base=None))

    assert str(raised.value) == (
        "Bad snapcraft.yaml content:\n"
        "- Snap base must be declared when type is not base, kernel or snapd"
    )


def test_expand_environment(new_dir, mocker):
    mocker.patch("platform.machine", return_value="aarch64")

    yaml_data = {
        "name": "test-env",
        "version": "1.2.3",
        "grade": "stable",
        "field0": "$CRAFT_PROJECT_NAME",
        "field1": "$SNAPCRAFT_PROJECT_NAME",
        "field2": "$SNAPCRAFT_PROJECT_VERSION",
        "field3": "$SNAPCRAFT_PROJECT_GRADE",
        "field4": "$CRAFT_ARCH_TRIPLET",
        "field5": "$SNAPCRAFT_ARCH_TRIPLET",
        "field6": "$CRAFT_TARGET_ARCH",
        "field8": "$SNAPCRAFT_TARGET_ARCH",
        "dirs": {
            "field9": ["$CRAFT_STAGE", "$SNAPCRAFT_STAGE"],
            "field10": ["${CRAFT_PRIME}", "${SNAPCRAFT_PRIME}"],
            "field11": ["$CRAFT_PROJECT_DIR", "$SNAPCRAFT_PROJECT_DIR"],
        },
        "field12": ["$CRAFT_PARALLEL_BUILD_COUNT", "$SNAPCRAFT_PARALLEL_BUILD_COUNT"],
    }
    parts_lifecycle._expand_environment(
        yaml_data, parallel_build_count=8, target_arch="arm64"
    )

    assert yaml_data == {
        "name": "test-env",
        "version": "1.2.3",
        "grade": "stable",
        "field0": "test-env",
        "field1": "test-env",
        "field2": "1.2.3",
        "field3": "stable",
        "field4": "aarch64-linux-gnu",
        "field5": "aarch64-linux-gnu",
        "field6": "arm64",
        "field8": "arm64",
        "dirs": {
            "field9": [f"{new_dir}/stage", f"{new_dir}/stage"],
            "field10": [f"{new_dir}/prime", f"{new_dir}/prime"],
            "field11": [f"{new_dir}", f"{new_dir}"],
        },
        "field12": ["8", "8"],
    }


def test_lifecycle_run_expand_snapcraft_vars(new_dir, mocker):
    mocker.patch("platform.machine", return_value="aarch64")
    mocker.patch(
        "snapcraft.parts.lifecycle.get_build_plan",
        return_value=[("arm64", "arm64")],
    )

    content = textwrap.dedent(
        """\
        name: mytest
        version: "0.1"
        base: core22
        summary: Environment expansion test
        description: Environment expansion test
        grade: stable
        confinement: strict

        apps:
          app:
            command: usr/$SNAPCRAFT_ARCH_TRIPLET/foo

        parts:
          part1:
            plugin: nil
            override-build: |
              touch $SNAPCRAFT_PART_INSTALL/foo
              chmod +x $SNAPCRAFT_PART_INSTALL/foo
            organize:
              foo: usr/$SNAPCRAFT_ARCH_TRIPLET/foo
        """
    )

    yaml_path = Path("snapcraft.yaml")
    yaml_path.write_text(content)

    parts_lifecycle.run(
        "prime",
        argparse.Namespace(
            parts=[],
            destructive_mode=True,
            use_lxd=False,
            provider=None,
            enable_manifest=False,
            manifest_image_information=None,
            bind_ssh=False,
            ua_token=None,
            build_for=None,
            debug=False,
        ),
    )

    assert Path(new_dir / "prime/usr/aarch64-linux-gnu/foo").is_file()

    meta_yaml = Path(new_dir / "prime/meta/snap.yaml").read_text()
    assert "command: usr/aarch64-linux-gnu/foo" in meta_yaml


def test_lifecycle_run_expand_craft_vars(new_dir, mocker):
    mocker.patch("platform.machine", return_value="aarch64")
    mocker.patch(
        "snapcraft.parts.lifecycle.get_build_plan",
        return_value=[("arm64", "arm64")],
    )

    content = textwrap.dedent(
        """\
        name: mytest
        version: "0.1"
        base: core22
        summary: Environment expansion test
        description: Environment expansion test
        grade: stable
        confinement: strict

        apps:
          app:
            command: usr/$CRAFT_ARCH_TRIPLET/foo

        parts:
          part1:
            plugin: nil
            override-build: |
              touch $CRAFT_PART_INSTALL/foo
              chmod +x $CRAFT_PART_INSTALL/foo
            organize:
              foo: usr/$CRAFT_ARCH_TRIPLET/foo
        """
    )

    yaml_path = Path("snapcraft.yaml")
    yaml_path.write_text(content)

    parts_lifecycle.run(
        "prime",
        argparse.Namespace(
            parts=[],
            destructive_mode=True,
            use_lxd=False,
            provider=None,
            enable_manifest=False,
            manifest_image_information=None,
            bind_ssh=False,
            ua_token=None,
            build_for=None,
            debug=False,
        ),
    )

    assert Path(new_dir / "prime/usr/aarch64-linux-gnu/foo").is_file()

    meta_yaml = Path(new_dir / "prime/meta/snap.yaml").read_text()
    assert "command: usr/aarch64-linux-gnu/foo" in meta_yaml


def test_lifecycle_run_permission_denied(new_dir):
    content = textwrap.dedent(
        """\
        name: mytest
        version: "0.1"
        base: core22
        summary: Permission error test
        description: Permission error test
        grade: stable
        confinement: strict

        parts:
          part1:
            plugin: nil
        """
    )

    yaml_path = Path("snapcraft.yaml")
    yaml_path.write_text(content)

    Path("prime/meta/").mkdir(parents=True)
    Path("prime/meta/snap.yaml").touch()
    Path("prime/meta/snap.yaml").chmod(0o000)

    with pytest.raises(errors.FilePermissionError) as raised:
        parts_lifecycle.run(
            "prime",
            argparse.Namespace(
                parts=[],
                destructive_mode=True,
                use_lxd=False,
                provider=None,
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                ua_token=None,
                build_for=None,
                debug=False,
            ),
        )

    error = raised.value
    assert str(error) == f"Permission denied in file {new_dir!s}/prime/meta/snap.yaml"
    assert error.resolution == (
        "Make sure the file is part of the current project "
        "and its permissions and ownership are correct."
    )


def test_lifecycle_run_in_provider_default(
    mock_get_instance_name,
    mock_instance,
    mock_provider,
    mocker,
    snapcraft_yaml,
    tmp_path,
):
    """Verify default calls made in `run_in_provider()`"""
    mock_base_configuration = Mock()
    mock_get_base_configuration = mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_base_configuration",
        return_value=mock_base_configuration,
    )
    mock_capture_logs_from_instance = mocker.patch(
        "snapcraft.parts.lifecycle.providers.capture_logs_from_instance"
    )
    mock_ensure_provider_is_available = mocker.patch(
        "snapcraft.parts.lifecycle.providers.ensure_provider_is_available"
    )
    mock_prepare_instance = mocker.patch(
        "snapcraft.parts.lifecycle.providers.prepare_instance"
    )
    mocker.patch("snapcraft.projects.Project.get_build_on", return_value="test-arch-1")
    mocker.patch("snapcraft.projects.Project.get_build_for", return_value="test-arch-2")

    expected_command = [
        "snapcraft",
        "test",
        "--verbosity=quiet",
        "--build-for",
        "test-arch-2",
    ]

    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    parts_lifecycle._run_in_provider(
        project=project,
        command_name="test",
        parsed_args=argparse.Namespace(
            use_lxd=False,
            debug=False,
            bind_ssh=False,
            http_proxy=None,
            https_proxy=None,
        ),
    )

    mock_ensure_provider_is_available.assert_called_once_with(mock_provider)
    mock_get_instance_name.assert_called_once_with(
        project_name="mytest",
        project_path=tmp_path,
        build_on="test-arch-1",
        build_for="test-arch-2",
    )
    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        instance_name="test-instance-name",
        http_proxy=None,
        https_proxy=None,
    )
    mock_provider.launched_environment.assert_called_with(
        project_name="mytest",
        project_path=ANY,
        base_configuration=mock_base_configuration,
        build_base="22.04",
        instance_name="test-instance-name",
        allow_unstable=False,
    )
    mock_prepare_instance.assert_called_with(
        instance=mock_instance, host_project_path=tmp_path, bind_ssh=False
    )
    mock_instance.execute_run.assert_called_once_with(
        expected_command, check=True, cwd=Path("/root/project")
    )
    mock_capture_logs_from_instance.assert_called_once()


@pytest.mark.parametrize(
    "emit_mode,verbosity",
    [
        (EmitterMode.VERBOSE, "--verbosity=verbose"),
        (EmitterMode.QUIET, "--verbosity=quiet"),
        (EmitterMode.BRIEF, "--verbosity=brief"),
        (EmitterMode.DEBUG, "--verbosity=debug"),
        (EmitterMode.TRACE, "--verbosity=trace"),
    ],
)
# pylint: disable-next=too-many-locals
def test_lifecycle_run_in_provider_all_options(
    mock_get_instance_name,
    mock_instance,
    mock_provider,
    mocker,
    snapcraft_yaml,
    tmp_path,
    emit_mode,
    verbosity,
):
    """Verify all project options are parsed in `run_in_provider()`."""
    mock_base_configuration = Mock()
    mock_get_base_configuration = mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_base_configuration",
        return_value=mock_base_configuration,
    )
    mock_capture_logs_from_instance = mocker.patch(
        "snapcraft.parts.lifecycle.providers.capture_logs_from_instance"
    )
    mock_ensure_provider_is_available = mocker.patch(
        "snapcraft.parts.lifecycle.providers.ensure_provider_is_available"
    )
    mock_prepare_instance = mocker.patch(
        "snapcraft.parts.lifecycle.providers.prepare_instance"
    )
    mocker.patch("snapcraft.projects.Project.get_build_on", return_value="test-arch-1")
    mocker.patch("snapcraft.projects.Project.get_build_for", return_value="test-arch-2")

    # build the expected command to be executed in the provider
    parts = ["test-part-1", "test-part-2"]
    output = "test-output"
    manifest_build_information = "test-build-info"
    ua_token = "test-ua-token"
    http_proxy = "1.2.3.4"
    https_proxy = "5.6.7.8"
    expected_command = (
        ["snapcraft", "test"]
        + parts
        + [
            "--output",
            output,
            verbosity,
            "--debug",
            "--shell",
            "--shell-after",
            "--enable-manifest",
            "--manifest-build-information",
            manifest_build_information,
            "--build-for",
            "test-arch-2",
            "--ua-token",
            ua_token,
            "--enable-experimental-ua-services",
        ]
    )

    # set emitter mode
    emit.set_mode(emit_mode)

    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    parts_lifecycle._run_in_provider(
        project=project,
        command_name="test",
        parsed_args=argparse.Namespace(
            parts=parts,
            output=output,
            destructive_mode=False,
            use_lxd=False,
            provider=None,
            enable_manifest=True,
            manifest_build_information=manifest_build_information,
            bind_ssh=True,
            ua_token=ua_token,
            enable_experimental_ua_services=True,
            build_for=None,
            debug=True,
            shell=True,
            shell_after=True,
            http_proxy=http_proxy,
            https_proxy=https_proxy,
        ),
    )

    mock_ensure_provider_is_available.assert_called_once_with(mock_provider)
    mock_get_instance_name.assert_called_once_with(
        project_name="mytest",
        project_path=tmp_path,
        build_on="test-arch-1",
        build_for="test-arch-2",
    )
    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        instance_name="test-instance-name",
        http_proxy="1.2.3.4",
        https_proxy="5.6.7.8",
    )
    mock_provider.launched_environment.assert_called_with(
        project_name="mytest",
        project_path=ANY,
        base_configuration=mock_base_configuration,
        build_base="22.04",
        instance_name="test-instance-name",
        allow_unstable=False,
    )
    mock_prepare_instance.assert_called_with(
        instance=mock_instance, host_project_path=tmp_path, bind_ssh=True
    )
    mock_instance.execute_run.assert_called_once_with(
        expected_command, check=True, cwd=Path("/root/project")
    )
    mock_capture_logs_from_instance.assert_called_once()


def test_lifecycle_run_in_provider_try(
    mock_get_instance_name,
    mock_instance,
    mock_provider,
    mocker,
    snapcraft_yaml,
    tmp_path,
):
    """Test that "snapcraft try" mounts the host's prime dir before priming in the instance"""
    mock_base_configuration = Mock()
    mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_base_configuration",
        return_value=mock_base_configuration,
    )
    mocker.patch("snapcraft.parts.lifecycle.providers.capture_logs_from_instance")
    mocker.patch("snapcraft.parts.lifecycle.providers.ensure_provider_is_available")
    mocker.patch("snapcraft.parts.lifecycle.providers.prepare_instance")
    mocker.patch("snapcraft.projects.Project.get_build_on", return_value="test-arch-1")
    mocker.patch("snapcraft.projects.Project.get_build_for", return_value="test-arch-2")

    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    parts_lifecycle._run_in_provider(
        project=project,
        command_name="try",
        parsed_args=argparse.Namespace(
            use_lxd=False,
            debug=False,
            bind_ssh=False,
            http_proxy=None,
            https_proxy=None,
        ),
    )

    expected_command = [
        "snapcraft",
        "try",
        "--verbosity=quiet",
        "--build-for",
        "test-arch-2",
    ]

    # Make sure the calls are made in the correct order: first the host 'prime' dir
    # is mounted, and _then_ the command is run in the instance.
    mock_instance.assert_has_calls(
        [
            call.mount(host_source=tmp_path / "prime", target=Path("/root/prime")),
            call.execute_run(expected_command, check=True, cwd=Path("/root/project")),
        ],
        any_order=False,
    )


def test_lifecycle_run_in_provider(
    mock_get_instance_name,
    mock_instance,
    mock_provider,
    mocker,
    snapcraft_yaml,
    tmp_path,
):
    """Verify snapcraft bases are handled properly when launching an instance."""
    mock_base_configuration = Mock()
    mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_base_configuration",
        return_value=mock_base_configuration,
    )
    mocker.patch("snapcraft.parts.lifecycle.providers.capture_logs_from_instance")
    mocker.patch("snapcraft.parts.lifecycle.providers.ensure_provider_is_available")
    mocker.patch("snapcraft.parts.lifecycle.providers.prepare_instance")
    mocker.patch("snapcraft.projects.Project.get_build_on")
    mocker.patch("snapcraft.projects.Project.get_build_for")

    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    parts_lifecycle._run_in_provider(
        project=project,
        command_name="test",
        parsed_args=argparse.Namespace(
            use_lxd=False,
            debug=False,
            bind_ssh=False,
            http_proxy=None,
            https_proxy=None,
        ),
    )

    mock_provider.launched_environment.assert_called_with(
        project_name="mytest",
        project_path=ANY,
        base_configuration=mock_base_configuration,
        build_base=BuilddBaseAlias.JAMMY.value,
        instance_name="test-instance-name",
        allow_unstable=False,
    )


def test_lifecycle_run_in_provider_devel_base(
    emitter,
    mock_get_instance_name,
    mock_instance,
    mock_provider,
    mocker,
    snapcraft_yaml,
    tmp_path,
):
    """Verify the `devel` base is handled properly when launching an instance."""
    mocker.patch("snapcraft.projects.Project.get_effective_base", return_value="devel")
    mock_base_configuration = Mock()
    mocker.patch(
        "snapcraft.parts.lifecycle.providers.get_base_configuration",
        return_value=mock_base_configuration,
    )
    mocker.patch("snapcraft.parts.lifecycle.providers.capture_logs_from_instance")
    mocker.patch("snapcraft.parts.lifecycle.providers.ensure_provider_is_available")
    mocker.patch("snapcraft.parts.lifecycle.providers.prepare_instance")
    mocker.patch("snapcraft.projects.Project.get_build_on")
    mocker.patch("snapcraft.projects.Project.get_build_for")

    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    parts_lifecycle._run_in_provider(
        project=project,
        command_name="test",
        parsed_args=argparse.Namespace(
            use_lxd=False,
            debug=False,
            bind_ssh=False,
            http_proxy=None,
            https_proxy=None,
        ),
    )

    emitter.assert_progress(
        "Running snapcraft with a devel instance is for testing purposes only.",
        permanent=True,
    )


@pytest.fixture
def minimal_yaml_data():
    return {
        "name": "name",
        "base": "core22",
        "confinement": "strict",
        "grade": "devel",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "parts": {"nil": {}},
    }


@pytest.mark.parametrize("key", ("build-packages", "build-snaps"))
@pytest.mark.parametrize("value", (["foo"], [{"on amd64": ["foo"]}]))
def test_root_packages(minimal_yaml_data, key, value):
    minimal_yaml_data[key] = value
    arch = get_host_architecture()

    assert parts_lifecycle.apply_yaml(
        minimal_yaml_data, build_on=arch, build_for=arch
    ) == {
        "name": "name",
        "base": "core22",
        "confinement": "strict",
        "grade": "devel",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "architectures": [Architecture(build_on=arch, build_for=arch)],
        "parts": {"nil": {}, "snapcraft/core": {"plugin": "nil", key: ["foo"]}},
    }


def test_get_build_plan_single_element_matching(snapcraft_yaml, mocker, new_dir):
    """Test get_build_plan with a single matching element."""
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": [{"build-on": "aarch64", "build-for": "aarch64"}],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert parts_lifecycle.get_build_plan(
        snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for=None)
    ) == [("aarch64", "aarch64")]


def test_get_build_plan_build_for_all(snapcraft_yaml, mocker, new_dir):
    """Test get_build_plan with `build-for: all`."""
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": [{"build-on": "aarch64", "build-for": "all"}],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert parts_lifecycle.get_build_plan(
        snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for=None)
    ) == [("aarch64", "all")]


def test_get_build_plan_with_matching_elements(snapcraft_yaml, mocker, new_dir):
    """The build plan should only contain builds where `build-on` matches
    the host architecture.
    """
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": [
            {"build-on": "aarch64", "build-for": "aarch64"},
            {"build-on": "aarch64", "build-for": "arm64"},
            {"build-on": "armhf", "build-for": "armhf"},
        ],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert parts_lifecycle.get_build_plan(
        snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for=None)
    ) == [
        ("aarch64", "aarch64"),
        ("aarch64", "arm64"),
    ]


def test_get_build_plan_list_without_matching_element(snapcraft_yaml, mocker, new_dir):
    """The build plan should be empty when no build has a `build-on` that matches
    the host architecture.
    """
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": ["powerpc", "armhf"],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert (
        parts_lifecycle.get_build_plan(
            snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for=None)
        )
        == []
    )


def test_get_build_plan_list_with_matching_element_and_env_var(
    snapcraft_yaml, mocker, monkeypatch, new_dir
):
    """The build plan should be filtered down when `SNAPCRAFT_BUILD_FOR` is defined."""
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": [
            {"build-on": "aarch64", "build-for": "aarch64"},
            {"build-on": "aarch64", "build-for": "armhf"},
        ],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert parts_lifecycle.get_build_plan(
        snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for="aarch64")
    ) == [("aarch64", "aarch64")]


def test_get_build_plan_list_without_matching_element_and_build_for_arg(
    snapcraft_yaml, mocker, monkeypatch, new_dir
):
    """The build plan should be empty when no plan has a matching `build_for`
    matching `SNAPCRAFT_BUILD_FOR.`"""
    mocker.patch(
        "snapcraft.parts.lifecycle.get_host_architecture", return_value="aarch64"
    )
    yaml_data = {
        "base": "core22",
        "architectures": [
            {"build-on": "aarch64", "build-for": "aarch64"},
            {"build-on": "aarch64", "build-for": "armhf"},
        ],
    }

    snapcraft_yaml_data = snapcraft_yaml(**yaml_data)

    assert (
        parts_lifecycle.get_build_plan(
            snapcraft_yaml_data, parsed_args=argparse.Namespace(build_for="arm64")
        )
        == []
    )


def test_patch_elf(snapcraft_yaml, mocker, new_dir):
    """Patch binaries if the ``enable-patchelf`` build attribute is defined."""
    run_patchelf_mock = mocker.patch("snapcraft.elf._patcher.Patcher._run_patchelf")
    shutil.copy("/bin/true", "elf.bin")
    callbacks.register_post_step(parts_lifecycle._patch_elf, step_list=[Step.PRIME])

    mocker.patch(
        "snapcraft.elf.elf_utils.get_dynamic_linker",
        return_value="/snap/core22/current/lib64/ld-linux-x86-64.so.2",
    )
    mocker.patch(
        "snapcraft.elf._patcher.Patcher.get_proposed_rpath",
        return_value=["/snap/core22/current/lib/x86_64-linux-gnu"],
    )
    mocker.patch("snapcraft.elf._patcher.Patcher.get_current_rpath", return_value=[])

    # don't load dependencies in a unit test
    mocker.patch.object(ElfFile, "load_dependencies")

    yaml_data = {
        "base": "core22",
        "confinement": "classic",
        "parts": {
            "p1": {
                "plugin": "dump",
                "source": ".",
                "build-attributes": ["enable-patchelf"],
            }
        },
    }
    project = Project.unmarshal(snapcraft_yaml(**yaml_data))

    parts_lifecycle._run_command(
        "pack",
        project=project,
        parse_info={},
        assets_dir=Path(),
        start_time=datetime.now(),
        parallel_build_count=1,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=False,
            shell_after=False,
            use_lxd=False,
            enable_manifest=False,
            ua_token=None,
            parts=["p1"],
        ),
    )

    assert run_patchelf_mock.mock_calls == [
        call(
            patchelf_args=[
                "--set-interpreter",
                "/snap/core22/current/lib64/ld-linux-x86-64.so.2",
            ],
            elf_file_path=new_dir / "prime/elf.bin",
        )
    ]
