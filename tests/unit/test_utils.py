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

import os
from pathlib import Path
from typing import List
from unittest.mock import call, patch

import pytest

from snapcraft import errors, utils


@pytest.fixture
def mock_isatty(mocker):
    yield mocker.patch("snapcraft.utils.sys.stdin.isatty", return_value=True)


@pytest.fixture
def mock_input(mocker):
    yield mocker.patch("snapcraft.utils.input", return_value="")


@pytest.fixture
def mock_is_managed_mode(mocker):
    yield mocker.patch("snapcraft.utils.is_managed_mode", return_value=False)


#####################
# Get Host Platform #
#####################


@pytest.mark.parametrize(
    "base,build_base,project_type,name,expected_base",
    [
        (None, "build_base", "base", "name", "build_base"),
        ("base", "build_base", "base", "name", "build_base"),
        (None, None, "base", "name", "name"),
        ("base", None, "base", "name", "name"),
        (None, None, "other", "name", None),
        ("base", "build_base", "other", "name", "build_base"),
        ("base", None, "other", "name", "base"),
        ("base", "devel", "other", "name", "base"),
        ("base", "devel", "base", "name", "devel"),
    ],
)
def test_get_effective_base(base, build_base, project_type, name, expected_base):
    result = utils.get_effective_base(
        base=base, build_base=build_base, project_type=project_type, name=name
    )
    assert result == expected_base


def test_get_effective_base_translate_devel():
    params = {
        "base": "core24",
        "build_base": "devel",
        "project_type": None,
        "name": "my-project",
    }

    translate_true = utils.get_effective_base(translate_devel=True, **params)
    assert translate_true == "core24"

    translate_false = utils.get_effective_base(translate_devel=False, **params)
    assert translate_false == "devel"


########################
# Parallel build count #
########################


def test_get_parallel_build_count(mocker):
    mocker.patch("os.sched_getaffinity", return_value=[1] * 13)
    assert utils.get_parallel_build_count() == 13


def test_get_parallel_build_count_no_affinity(mocker):
    mocker.patch("os.sched_getaffinity", side_effect=AttributeError)
    mocker.patch("multiprocessing.cpu_count", return_value=17)
    assert utils.get_parallel_build_count() == 17


def test_get_parallel_build_count_disable(mocker):
    mocker.patch("os.sched_getaffinity", side_effect=AttributeError)
    mocker.patch("multiprocessing.cpu_count", side_effect=NotImplementedError)
    assert utils.get_parallel_build_count() == 1


@pytest.mark.parametrize(
    "max_count,count",
    [("", 13), ("xx", 13), ("0", 13), ("1", 1), ("8", 8), ("13", 13), ("14", 13)],
)
def test_get_parallel_build_count_limited(mocker, max_count, count):
    mocker.patch("os.sched_getaffinity", return_value=[1] * 13)
    mocker.patch.dict(os.environ, {"SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT": max_count})
    assert utils.get_parallel_build_count() == count


#################
# Humanize List #
#################


@pytest.mark.parametrize(
    "items,conjunction,expected",
    (
        ([], "and", ""),
        (["foo"], "and", "'foo'"),
        (["foo", "bar"], "and", "'bar' and 'foo'"),
        (["foo", "bar", "baz"], "and", "'bar', 'baz', and 'foo'"),
        (["foo", "bar", "baz", "qux"], "and", "'bar', 'baz', 'foo', and 'qux'"),
        ([], "or", ""),
        (["foo"], "or", "'foo'"),
        (["foo", "bar"], "or", "'bar' or 'foo'"),
        (["foo", "bar", "baz"], "or", "'bar', 'baz', or 'foo'"),
        (["foo", "bar", "baz", "qux"], "or", "'bar', 'baz', 'foo', or 'qux'"),
    ),
)
def test_humanize_list(items, conjunction, expected):
    assert utils.humanize_list(items, conjunction) == expected


def test_humanize_list_sorted():
    """Verify `sort` parameter."""
    input_list = ["z", "a", "m test", "1"]

    # unsorted list is in the same order as the original list
    expected_list_unsorted = "'z', 'a', 'm test', and '1'"

    # sorted list is sorted alphanumerically
    expected_list_sorted = "'1', 'a', 'm test', and 'z'"

    assert utils.humanize_list(input_list, "and") == expected_list_sorted
    assert utils.humanize_list(input_list, "and", sort=True) == expected_list_sorted
    assert utils.humanize_list(input_list, "and", sort=False) == expected_list_unsorted


#################
# Library Paths #
#################


@pytest.mark.parametrize(
    "lib_dirs,expected_env",
    [
        (["lib"], "$SNAP/lib"),
        (["lib", "usr/lib"], "$SNAP/lib:$SNAP/usr/lib"),
        (
            ["lib/i286-none-none", "usr/lib/i286-none-none"],
            "$SNAP/lib:$SNAP/usr/lib:$SNAP/lib/i286-none-none:$SNAP/usr/lib/i286-none-none",
        ),
    ],
)
def test_get_ld_library_paths(tmp_path, lib_dirs, expected_env):
    for d in lib_dirs:
        (tmp_path / d).mkdir(parents=True)

    expected_env = (
        f"${{SNAP_LIBRARY_PATH}}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}:{expected_env}"
    )
    assert utils.get_ld_library_paths(tmp_path, "i286-none-none") == expected_env


@pytest.mark.parametrize(
    ["lib_dirs", "expected_env"],
    [
        (["lib"], "$SNAP/lib"),
        (["lib", "usr/lib"], "$SNAP/lib:$SNAP/usr/lib"),
        (["lib/i286-none-none", "usr/lib/i286-none-none"], "$SNAP/lib:$SNAP/usr/lib"),
    ],
)
def test_get_ld_library_paths_no_architecture(tmp_path, lib_dirs, expected_env):
    """Do not include architecture-specfic paths if an architecture is not provided."""
    for lib_dir in lib_dirs:
        (tmp_path / lib_dir).mkdir(parents=True)

    env = utils.get_ld_library_paths(tmp_path, None)

    assert env == (
        f"${{SNAP_LIBRARY_PATH}}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}:{expected_env}"
    )


#################
# Get host tool #
#################


def test_get_host_tool_finds_command(mocker):
    mocker.patch("shutil.which", return_value="/usr/bin/foo")

    assert utils.get_host_tool(command_name="foo") == "/usr/bin/foo"


def test_get_host_tool_failure(mocker):
    mocker.patch("shutil.which", return_value=None)

    with pytest.raises(errors.SnapcraftError) as raised:
        utils.get_host_tool(command_name="foo")

    assert str(raised.value) == "A tool snapcraft depends on could not be found: 'foo'"


#################
# Get snap tool #
#################


@pytest.fixture()
def fake_exists(mocker):
    """Fakely return True when checking for preconfigured paths."""

    class _FileCheck:
        def __init__(self) -> None:
            self._original_exists = os.path.exists
            self.paths: List[str] = []

        def exists(self, path: str) -> bool:
            if Path(path) in self.paths:
                return True
            return self._original_exists(path)

    file_checker = _FileCheck()
    mocker.patch("os.path.exists", new=file_checker.exists)

    yield file_checker


@pytest.fixture()
def in_snap(mocker):
    """Simulate being run from within the context of the Snapcraft snap."""
    mocker.patch.dict(
        os.environ,
        {
            "SNAP": "/snap/snapcraft/current",
            "SNAP_NAME": "snapcraft",
            "SNAP_VERSION": "7.0",
        },
    )


_BIN_PATHS = [
    "usr/local/sbin",
    "usr/local/bin",
    "usr/sbin",
    "usr/bin",
    "sbin",
    "bin",
]


@pytest.mark.parametrize("bin_path", _BIN_PATHS)
def test_get_snap_tool_from_host_path(mocker, bin_path, fake_exists):
    abs_tool_path = Path("/") / bin_path / "tool-command"
    fake_exists.paths = [abs_tool_path]
    mocker.patch("shutil.which", return_value=abs_tool_path.as_posix())

    assert utils.get_snap_tool("tool-command") == abs_tool_path.as_posix()


@pytest.mark.parametrize("bin_path", _BIN_PATHS)
def test_get_snap_tool_from_snapcraft_snap_path(bin_path, in_snap, fake_exists):
    abs_tool_path = Path("/snap/snapcraft/current") / bin_path / "tool-command"
    fake_exists.paths = [abs_tool_path]

    assert utils.get_snap_tool("tool-command") == abs_tool_path.as_posix()


def test_get_snap_tool_path_fails():
    with pytest.raises(errors.SnapcraftError) as raised:
        utils.get_snap_tool("non-existent-tool-command")

    assert str(raised.value) == (
        "A tool snapcraft depends on could not be found: 'non-existent-tool-command'"
    )


###################
# Process Version #
###################


def test_process_version_empty(mocker):
    """``version:None`` raises an error."""
    with pytest.raises(ValueError):
        utils.process_version(None)


def test_process_version_dirty(mocker):
    """A version string should be returned unmodified."""
    assert utils.process_version("1.2.3") == "1.2.3"


def test_process_version_git(mocker):
    """``version:git`` must be correctly handled."""
    mocker.patch(
        "craft_parts.sources.git_source.GitSource.generate_version",
        return_value="1.2.3-dirty",
    )

    assert utils.process_version("git") == "1.2.3-dirty"


########################
# Is running from snap #
########################


@pytest.mark.parametrize(
    "snap_name,snap,result",
    [
        (None, None, False),
        (None, "/snap/snapcraft/x1", False),
        ("snapcraft", None, False),
        ("snapcraft", "/snap/snapcraft/x1", True),
    ],
)
def test_is_snapcraft_running_from_snap(monkeypatch, snap_name, snap, result):
    if snap_name is None:
        monkeypatch.delenv("SNAP_NAME", raising=False)
    else:
        monkeypatch.setenv("SNAP_NAME", snap_name)

    if snap is None:
        monkeypatch.delenv("SNAP", raising=False)
    else:
        monkeypatch.setenv("SNAP", snap)

    assert utils.is_snapcraft_running_from_snap() == result


#####################
# Confirm with user #
#####################


def test_confirm_with_user_defaults_with_tty(mock_input, mock_isatty):
    mock_input.return_value = ""
    mock_isatty.return_value = True

    assert utils.confirm_with_user("prompt", default=True) is True
    assert mock_input.mock_calls == [call("prompt [Y/n]: ")]
    mock_input.reset_mock()

    assert utils.confirm_with_user("prompt", default=False) is False
    assert mock_input.mock_calls == [call("prompt [y/N]: ")]


def test_confirm_with_user_defaults_without_tty(mock_input, mock_isatty):
    mock_isatty.return_value = False

    assert utils.confirm_with_user("prompt", default=True) is True
    assert utils.confirm_with_user("prompt", default=False) is False

    assert mock_input.mock_calls == []


@pytest.mark.parametrize(
    "user_input,expected",
    [
        ("y", True),
        ("Y", True),
        ("yes", True),
        ("YES", True),
        ("n", False),
        ("N", False),
        ("no", False),
        ("NO", False),
        ("other", False),
        ("", False),
    ],
)
def test_confirm_with_user(user_input, expected, mock_input, mock_isatty):
    """Verify different inputs are accepted with a tendency to interpret as 'no'."""
    mock_input.return_value = user_input

    assert utils.confirm_with_user("prompt") == expected
    assert mock_input.mock_calls == [call("prompt [y/N]: ")]


def test_confirm_with_user_errors_in_managed_mode(mock_is_managed_mode):
    mock_is_managed_mode.return_value = True

    with pytest.raises(RuntimeError):
        utils.confirm_with_user("prompt")


def test_confirm_with_user_pause_emitter(mock_isatty, emitter):
    """The emitter should be paused when using the terminal."""
    mock_isatty.return_value = True

    def fake_input(prompt):
        """Check if the Emitter is paused."""
        assert emitter.paused
        return ""

    with patch("snapcraft.utils.input", fake_input):
        utils.confirm_with_user("prompt")
