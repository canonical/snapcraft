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

import pytest

from snapcraft import linters, projects
from snapcraft.elf import elf_utils
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.linters.classic_linter import ClassicLinter
from snapcraft.meta import snap_yaml


def setup_function():
    elf_utils.get_elf_files.cache_clear()


@pytest.mark.parametrize(
    "confinement,stage_libc,text",
    [
        ("classic", False, "Snap confinement is set to classic."),
        ("strict", True, "Snap contains staged libc."),
        ("strict", False, None),
    ],
)
def test_classic_linter(mocker, new_dir, confinement, stage_libc, text):
    shutil.copy("/bin/true", "elf.bin")
    shutil.copy("/lib/x86_64-linux-gnu/libdl.so.2", "elf.lib")

    if stage_libc:
        Path("lib64").mkdir()
        Path("lib64/ld-linux-x86-64.so.2").touch()

    mocker.patch("snapcraft.linters.linters.LINTERS", {"classic": ClassicLinter})
    mocker.patch(
        "snapcraft.elf._elf_file._determine_libraries",
        return_value={
            "libc.so.6": "/snap/core22/current/lib/x86_64-linux-gnu/libc.so.6"
        },
    )
    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": confinement,
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    if confinement == "classic" or stage_libc:
        snap_name = "mytest" if stage_libc else "core22"
        assert issues == [
            LinterIssue(
                name="classic",
                result=LinterResult.OK,
                text=text,
            ),
            LinterIssue(
                name="classic",
                result=LinterResult.WARNING,
                filename="elf.bin",
                text=(
                    f"ELF interpreter should be set to "
                    f"'/snap/{snap_name}/current/lib64/ld-linux-x86-64.so.2'."
                ),
                url="https://snapcraft.io/docs/linters-classic",
            ),
            LinterIssue(
                name="classic",
                result=LinterResult.WARNING,
                filename="elf.bin",
                text="ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'.",
                url="https://snapcraft.io/docs/linters-classic",
            ),
            LinterIssue(
                name="classic",
                result=LinterResult.WARNING,
                filename="elf.lib",
                text="ELF rpath should be set to '/snap/core22/current/lib/x86_64-linux-gnu'.",
                url="https://snapcraft.io/docs/linters-classic",
            ),
        ]
    else:
        assert issues == []


def test_classic_linter_filter(mocker, new_dir):
    shutil.copy("/bin/true", "elf.bin")

    mocker.patch("snapcraft.linters.linters.LINTERS", {"classic": ClassicLinter})
    mocker.patch(
        "snapcraft.elf._elf_file._determine_libraries",
        return_value={
            "libc.so.6": "/snap/core22/current/lib/x86_64-linux-gnu/libc.so.6"
        },
    )
    yaml_data = {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "classic",
        "parts": {},
    }

    project = projects.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir, lint=projects.Lint(ignore=[{"classic": ["elf.*"]}])
    )
    assert issues == [
        LinterIssue(
            name="classic",
            result=LinterResult.OK,
            text="Snap confinement is set to classic.",
        ),
    ]
