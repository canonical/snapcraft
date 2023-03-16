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

import shutil
from pathlib import Path
from unittest.mock import Mock

import pytest

from snapcraft import linters, projects
from snapcraft.elf import _elf_file, elf_utils
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.linters.library_linter import LibraryLinter
from snapcraft.meta import snap_yaml


def setup_function():
    elf_utils.get_elf_files.cache_clear()


def test_library_linter_missing_library(mocker, new_dir):
    """Verify missing libraries are caught by the linter."""
    shutil.copy("/bin/true", "elf.bin")

    mocker.patch("snapcraft.linters.linters.LINTERS", {"library": LibraryLinter})
    mocker.patch(
        "snapcraft.elf._elf_file._determine_libraries",
        return_value={
            "libfoo.so.1": "/prime/lib/x86_64-linux-gnu/libfoo.so.1",
            "libbar.so.5": "/prime/lib/x86_64-linux-gnu/libbar.so.5",
        },
    )
    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(
        project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    issues = linters.run_linters(new_dir, lint=None)
    assert issues == [
        LinterIssue(
            name="library",
            result=LinterResult.WARNING,
            filename="elf.bin",
            text="missing dependency 'libbar.so.5'.",
            url="https://snapcraft.io/docs/linters-library",
        ),
        LinterIssue(
            name="library",
            result=LinterResult.WARNING,
            filename="elf.bin",
            text="missing dependency 'libfoo.so.1'.",
            url="https://snapcraft.io/docs/linters-library",
        ),
    ]


def test_library_linter_unused_library(mocker, new_dir):
    """Verify unused libraries are caught by the linter."""
    # mock an elf file
    mock_elf_file = Mock(spec=_elf_file.ElfFile)
    mock_elf_file.soname = ""
    mock_elf_file.path = Path("elf.bin")
    mock_elf_file.load_dependencies.return_value = []

    # mock a library
    mock_library = Mock(spec=_elf_file.ElfFile)
    mock_library.soname = "libfoo.so"
    mock_library.path = Path("lib/libfoo.so")
    mock_library.load_dependencies.return_value = []

    mocker.patch(
        "snapcraft.linters.library_linter.elf_utils.get_elf_files",
        return_value=[mock_elf_file, mock_library],
    )

    mocker.patch("snapcraft.linters.library_linter.ElfFile", return_value=mock_library)
    mocker.patch("snapcraft.linters.library_linter.Path.is_file", return_value=True)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"library": LibraryLinter})

    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(
        project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    issues = linters.run_linters(new_dir, lint=None)
    assert issues == [
        LinterIssue(
            name="library",
            result=LinterResult.WARNING,
            filename="libfoo.so",
            text="unused library 'lib/libfoo.so'.",
            url="https://snapcraft.io/docs/linters-library",
        ),
    ]


@pytest.mark.parametrize(
    "filter_name",
    ("library", "missing-library"),
)
def test_library_linter_filter_missing_library(mocker, new_dir, filter_name):
    """Verify missing libraries can be filtered out."""
    shutil.copy("/bin/true", "elf.bin")

    mocker.patch("snapcraft.linters.linters.LINTERS", {"library": LibraryLinter})
    mocker.patch(
        "snapcraft.elf._elf_file._determine_libraries",
        return_value={
            "libfoo.so.1": "/prime/lib/x86_64-linux-gnu/libfoo.so.1",
            "libbar.so.5": "/prime/lib/x86_64-linux-gnu/libbar.so.5",
        },
    )
    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(
        project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    issues = linters.run_linters(
        new_dir, lint=projects.Lint(ignore=[{filter_name: ["elf.*"]}])
    )
    assert issues == []


@pytest.mark.parametrize(
    "filter_name",
    ("library", "unused-library"),
)
def test_library_linter_filter_unused_library(mocker, new_dir, filter_name):
    """Verify unused libraries can be filtered out."""
    # mock an elf file
    mock_elf_file = Mock(spec=_elf_file.ElfFile)
    mock_elf_file.soname = ""
    mock_elf_file.path = Path("elf.bin")
    mock_elf_file.load_dependencies.return_value = []

    # mock a library
    mock_library = Mock(spec=_elf_file.ElfFile)
    mock_library.soname = "libfoo.so"
    mock_library.path = Path("lib/libfoo.so")
    mock_library.load_dependencies.return_value = []

    mocker.patch(
        "snapcraft.linters.library_linter.elf_utils.get_elf_files",
        return_value=[mock_elf_file, mock_library],
    )

    mocker.patch("snapcraft.linters.library_linter.Path.is_file", return_value=True)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"library": LibraryLinter})

    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(
        project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    issues = linters.run_linters(
        new_dir, lint=projects.Lint(ignore=[{filter_name: ["lib/libfoo.*"]}])
    )
    assert issues == []


def test_library_linter_mixed_filters(mocker, new_dir):
    """Check that filtering *missing* libraries does not affect *unused* ones."""

    # mock a library
    mock_library = Mock(spec=_elf_file.ElfFile)
    mock_library.soname = "libfoo.so"
    mock_library.path = Path("lib/libfoo.so")
    mock_library.load_dependencies.return_value = []

    mocker.patch(
        "snapcraft.linters.library_linter.elf_utils.get_elf_files",
        return_value=[mock_library],
    )

    mocker.patch("snapcraft.linters.library_linter.ElfFile", return_value=mock_library)
    mocker.patch("snapcraft.linters.library_linter.Path.is_file", return_value=True)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"library": LibraryLinter})

    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(
        project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    # lib/libfoo.so is an *unused* library, but here we filter out *missing* library
    # issues for this path.
    issues = linters.run_linters(
        new_dir, lint=projects.Lint(ignore=[{"missing-library": ["lib/libfoo.*"]}])
    )
    # The "unused library" issue must be generated.
    assert issues == [
        LinterIssue(
            name="library",
            result=LinterResult.WARNING,
            filename="libfoo.so",
            text="unused library 'lib/libfoo.so'.",
            url="https://snapcraft.io/docs/linters-library",
        ),
    ]


@pytest.mark.parametrize(
    "path, expected_result",
    [
        # files not in library paths should return False
        (Path("/test/file"), False),
        (Path("/lib/x86_64-linux-gnu/subdir/libtest.so"), False),
        (Path("/usr/lib/x86_64-linux-gnu/subdir/libtest.so"), False),
        (Path("/usr/lib/arm-linux-gnueabihf/subdir/libtest.so"), False),
        # files in library paths should return True
        (Path("/lib/libtest.so"), True),
        (Path("/usr/lib/libtest.so"), True),
        (Path("/usr/lib32/libtest.so"), True),
        (Path("/usr/lib64/libtest.so"), True),
        (Path("/usr/lib/x86_64-linux-gnu/libtest.so"), True),
        (Path("/root/stage/lib/libtest.so"), True),
        (Path("/root/stage/lib/x86_64-linux-gnu/libtest.so"), True),
    ],
)
def test_is_library_path(mocker, path, expected_result):
    """Check if filepaths are inside a library directory."""
    mocker.patch("snapcraft.linters.library_linter.Path.is_file", return_value=True)
    linter = LibraryLinter(name="library", snap_metadata=Mock(), lint=None)
    result = linter._is_library_path(path=path)

    assert result == expected_result


def test_is_library_path_directory(mocker):
    """Running `is_library_path()` on a directory should always return False."""
    mocker.patch("snapcraft.linters.library_linter.Path.is_file", return_value=False)
    linter = LibraryLinter(name="library", snap_metadata=Mock(), lint=None)
    result = linter._is_library_path(path=Path("/test/dir"))

    assert not result
