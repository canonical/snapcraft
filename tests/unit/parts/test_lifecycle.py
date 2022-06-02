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

import argparse
import textwrap
from pathlib import Path
from typing import Any, Dict
from unittest.mock import PropertyMock, call

import pytest
from craft_parts import Action, Step, callbacks

from snapcraft import errors
from snapcraft.parts import lifecycle as parts_lifecycle
from snapcraft.parts.update_metadata import update_project_metadata
from snapcraft.projects import MANDATORY_ADOPTABLE_FIELDS, Project

# pylint: disable=too-many-lines

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


@pytest.fixture
def snapcraft_yaml(new_dir):
    def write_file(
        *, base: str, filename: str = "snap/snapcraft.yaml"
    ) -> Dict[str, Any]:
        content = textwrap.dedent(
            f"""
            name: mytest
            version: '0.1'
            base: {base}
            summary: Just some test data
            description: This is just some test data.
            grade: stable
            confinement: strict

            parts:
              part1:
                plugin: nil
            """
        )
        yaml_path = Path(filename)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(content)

        return {
            "name": "mytest",
            "title": None,
            "base": base,
            "compression": "xz",
            "version": "0.1",
            "contact": None,
            "donation": None,
            "issues": None,
            "source-code": None,
            "website": None,
            "summary": "Just some test data",
            "description": "This is just some test data.",
            "type": None,
            "confinement": "strict",
            "icon": None,
            "layout": None,
            "license": None,
            "grade": "stable",
            "architectures": [],
            "package-repositories": [],
            "assumes": [],
            "hooks": None,
            "passthrough": None,
            "apps": None,
            "plugs": None,
            "slots": None,
            "parts": {"part1": {"plugin": "nil"}},
            "epoch": None,
        }

    yield write_file


@pytest.fixture
def project_vars(mocker):
    yield mocker.patch(
        "snapcraft.parts.PartsLifecycle.project_vars",
        new_callable=PropertyMock,
        return_value={"version": "0.1", "grade": "stable"},
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


@pytest.mark.parametrize("filename", _SNAPCRAFT_YAML_FILENAMES)
def test_snapcraft_yaml_load(new_dir, snapcraft_yaml, filename, mocker):
    """Snapcraft.yaml should be parsed as a valid yaml file."""
    yaml_data = snapcraft_yaml(base="core22", filename=filename)
    run_command_mock = mocker.patch("snapcraft.parts.lifecycle._run_command")
    mocker.patch("snapcraft.utils.get_parallel_build_count", return_value=5)

    parts_lifecycle.run(
        "pull",
        argparse.Namespace(
            parts=["part1"], destructive_mode=True, use_lxd=False, provider=None
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
            parsed_args=argparse.Namespace(
                parts=["part1"], destructive_mode=True, use_lxd=False, provider=None
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
    assert str(raised.value) == "base is not core22"


@pytest.mark.parametrize(
    "cmd,step",
    [
        ("pull", "pull"),
        ("build", "build"),
        ("stage", "stage"),
        ("prime", "prime"),
    ],
)
@pytest.mark.parametrize("debug_shell", [None, "debug", "shell", "shell_after"])
def test_lifecycle_run_command_step(
    cmd, step, debug_shell, snapcraft_yaml, project_vars, new_dir, mocker
):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.meta.snap_yaml.write")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")

    parsed_args = argparse.Namespace(
        debug=False,
        destructive_mode=True,
        shell=False,
        shell_after=False,
        use_lxd=False,
        parts=[],
    )

    if debug_shell:
        setattr(parsed_args, debug_shell, True)

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        parallel_build_count=8,
        parsed_args=parsed_args,
    )

    call_args = {"debug": False, "shell": False, "shell_after": False}
    if debug_shell:
        call_args[debug_shell] = True

    assert run_mock.mock_calls == [call(step, **call_args)]
    assert pack_mock.mock_calls == []


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_run_command_pack(cmd, snapcraft_yaml, project_vars, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.meta.snap_yaml.write")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")

    parts_lifecycle._run_command(
        cmd,
        project=project,
        parse_info={},
        assets_dir=Path(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=False,
            shell_after=False,
            use_lxd=False,
            parts=[],
        ),
    )

    assert run_mock.mock_calls == [
        call("prime", debug=False, shell=False, shell_after=False)
    ]
    assert pack_mock.mock_calls == [
        call(new_dir / "prime", output=None, compression="xz")
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_destructive_mode(
    cmd, snapcraft_yaml, project_vars, new_dir, mocker
):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    mocker.patch("snapcraft.meta.snap_yaml.write")
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
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=False,
            shell_after=False,
            use_lxd=False,
            parts=[],
        ),
    )

    assert run_in_provider_mock.mock_calls == []
    assert run_mock.mock_calls == [
        call("prime", debug=False, shell=False, shell_after=False)
    ]
    assert pack_mock.mock_calls == [
        call(new_dir / "home/prime", output=None, compression="xz")
    ]


@pytest.mark.parametrize("cmd", ["pack", "snap"])
def test_lifecycle_pack_managed(cmd, snapcraft_yaml, project_vars, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_in_provider_mock = mocker.patch("snapcraft.parts.lifecycle._run_in_provider")
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    mocker.patch("snapcraft.meta.snap_yaml.write")
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
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=False,
            shell=False,
            shell_after=False,
            use_lxd=False,
            parts=[],
        ),
    )

    assert run_in_provider_mock.mock_calls == []
    assert run_mock.mock_calls == [
        call("prime", debug=False, shell=False, shell_after=False)
    ]
    assert pack_mock.mock_calls == [
        call(new_dir / "home/prime", output=None, compression="xz")
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
                parts=[],
            ),
        )

    assert str(raised.value) == (
        "error setting grade: unexpected value; permitted: 'stable', 'devel'"
    )
    assert run_mock.mock_calls == [
        call("prime", debug=False, shell=False, shell_after=False)
    ]
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


def test_lifecycle_run_command_clean(snapcraft_yaml, project_vars, new_dir, mocker):
    """Clean provider project when called without parts."""
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    clean_mock = mocker.patch(
        "snapcraft.providers.LXDProvider.clean_project_environments",
        return_value=["instance-name"],
    )

    parts_lifecycle._run_command(
        "clean",
        project=project,
        parse_info={},
        assets_dir=Path(),
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            destructive_mode=False,
            use_lxd=False,
            parts=None,
        ),
    )

    assert clean_mock.mock_calls == [call(project_name="mytest", project_path=new_dir)]


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
    mock_shell = mocker.patch("subprocess.run")
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    with pytest.raises(errors.PartsLifecycleError):
        parts_lifecycle._run_command(
            cmd,
            project=project,
            parse_info={},
            assets_dir=Path(),
            parallel_build_count=8,
            parsed_args=argparse.Namespace(
                directory=None,
                output=None,
                debug=True,
                destructive_mode=True,
                shell=False,
                shell_after=False,
                use_lxd=False,
                parts=["part1"],
            ),
        )

    assert mock_shell.mock_calls == [call(["bash"], check=False, cwd=None)]


@pytest.mark.parametrize("cmd", ["pull", "build", "stage", "prime"])
def test_lifecycle_shell(snapcraft_yaml, cmd, new_dir, mocker):
    """Adoptable fields shouldn't be empty after adoption."""
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
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=True,
            shell_after=False,
            use_lxd=False,
            parts=["part1"],
        ),
    )

    expected_last_step = None
    if cmd == "build":
        expected_last_step = Step.OVERLAY
    if cmd == "stage":
        expected_last_step = Step.BUILD
    if cmd == "prime":
        expected_last_step = Step.STAGE

    assert last_step == expected_last_step
    assert mock_shell.mock_calls == [call(["bash"], check=False, cwd=None)]


@pytest.mark.parametrize("cmd", ["pull", "build", "stage", "prime"])
def test_lifecycle_shell_after(snapcraft_yaml, cmd, new_dir, mocker):
    """Adoptable fields shouldn't be empty after adoption."""
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
        parallel_build_count=8,
        parsed_args=argparse.Namespace(
            directory=None,
            output=None,
            debug=False,
            destructive_mode=True,
            shell=False,
            shell_after=True,
            use_lxd=False,
            parts=["part1"],
        ),
    )

    expected_last_step = Step.PULL
    if cmd == "build":
        expected_last_step = Step.BUILD
    if cmd == "stage":
        expected_last_step = Step.STAGE
    if cmd == "prime":
        expected_last_step = Step.PRIME

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
    parse_info = parts_lifecycle._extract_parse_info(yaml_data)
    assert yaml_data == {"name": "foo", "parts": {"p1": {"plugin": "nil"}, "p2": {}}}
    assert parse_info == {"p1": "foo/metadata.xml"}


def test_get_snap_project_no_base(snapcraft_yaml, new_dir):
    with pytest.raises(errors.ProjectValidationError) as raised:
        Project.unmarshal(snapcraft_yaml(base=None))

    assert str(raised.value) == (
        "Bad snapcraft.yaml content:\n"
        "- Snap base must be declared when type is not base, kernel or snapd"
    )


def test_get_snap_project_with_base(snapcraft_yaml):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    assert parts_lifecycle._get_extra_build_snaps(project) == ["core22"]


def test_get_snap_project_with_content_plugs(snapcraft_yaml, new_dir):
    yaml_data = {
        "name": "mytest",
        "version": "0.1",
        "base": "core22",
        "summary": "Just some test data",
        "description": "This is just some test data.",
        "grade": "stable",
        "confinement": "strict",
        "parts": {"part1": {"plugin": "nil"}},
        "plugs": {
            "test-plug-1": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-1",
            },
            "test-plug-2": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-2",
            },
        },
    }

    project = Project(**yaml_data)

    assert parts_lifecycle._get_extra_build_snaps(project) == [
        "core22",
        "test-snap-1",
        "test-snap-2",
    ]


def test_get_snap_project_with_content_plugs_does_not_add_extension(
    snapcraft_yaml, new_dir
):
    yaml_data = {
        "name": "mytest",
        "version": "0.1",
        "base": "core22",
        "summary": "Just some test data",
        "description": "This is just some test data.",
        "grade": "stable",
        "confinement": "strict",
        "plugs": {
            "test-plug-1": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-1",
            },
            "test-plug-2": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-2",
            },
        },
        "parts": {
            "part1": {"plugin": "nil", "build-snaps": ["test-snap-2", "test-snap-3"]}
        },
    }

    project = Project(**yaml_data)

    assert parts_lifecycle._get_extra_build_snaps(project) == [
        "core22",
        "test-snap-1",
    ]


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
    parts_lifecycle._expand_environment(yaml_data, parallel_build_count=8)

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
            parts=[], destructive_mode=True, use_lxd=False, provider=None, debug=False
        ),
    )

    assert Path(new_dir / "prime/usr/aarch64-linux-gnu/foo").is_file()

    meta_yaml = Path(new_dir / "prime/meta/snap.yaml").read_text()
    assert "command: usr/aarch64-linux-gnu/foo" in meta_yaml


def test_lifecycle_run_expand_craft_vars(new_dir, mocker):
    mocker.patch("platform.machine", return_value="aarch64")

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
            parts=[], destructive_mode=True, use_lxd=False, provider=None, debug=False
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
                debug=False,
            ),
        )

    error = raised.value
    assert str(error) == f"Permission denied in file {new_dir!s}/prime/meta/snap.yaml"
    assert error.resolution == (
        "Make sure the file is part of the current project "
        "and its permissions and ownership are correct."
    )
