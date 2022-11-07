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

import shutil
from pathlib import Path

from snapcraft import linters, projects
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.linters.library_linter import LibraryLinter
from snapcraft.meta import snap_yaml


def test_library_linter(mocker, new_dir):
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


def test_library_linter_filter(mocker, new_dir):
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
        new_dir, lint=projects.Lint(ignore=[{"library": ["elf.*"]}])
    )
    assert issues == []
