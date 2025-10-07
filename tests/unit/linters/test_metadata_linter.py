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

from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from snapcraft import linters, models
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.meta import snap_yaml


@pytest.fixture
def snap_yaml_data():
    def _snap_yaml_data(**kwargs):
        return {
            "name": "mytest",
            "adopt-info": "mytest",
            "base": "core22",
            "version": "1.29.3",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "icon": "icon.png",
            "contact": "Hello@h.com",
            "donation": [
                "https://donate1.com",
            ],
            "parts": {},
            "apps": {
                "app1": {
                    "command": "bin/mytest",
                },
            },
            **kwargs,
        }

    return _snap_yaml_data


def test_metadata_linter_missing_metadata(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    assert issues == [
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'title' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#title",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'license' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#license",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'issues' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#issues",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'source-code' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#source-code",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'website' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#website",
        ),
    ]


def test_metadata_linter_ignore_all_issues(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=models.Lint(ignore=["metadata"]))
    assert len(issues) == 0


def test_metadata_linter_ignore_some_issues(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir, lint=models.Lint(ignore=[{"metadata": ["title"]}])
    )
    assert issues == [
        LinterIssue(
            name="metadata",
            result=LinterResult.IGNORED,
            text="Metadata field 'title' is ignored.",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'license' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#license",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'issues' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#issues",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'source-code' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#source-code",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'website' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#website",
        ),
    ]


@pytest.mark.parametrize(
    ("lint_source", "specify_source", "linter_issue"),
    [
        pytest.param(
            True,
            False,
            LinterIssue(
                name="metadata",
                result=LinterResult.INFO,
                text="Metadata field 'source-code' is missing or empty.",
                url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#source-code",
            ),
            id="lint-missing",
        ),
        pytest.param(
            True,
            True,
            None,
            id="lint-happy",
        ),
        pytest.param(
            False,
            False,
            LinterIssue(
                name="metadata",
                result=LinterResult.IGNORED,
                text="Metadata field 'source-code' is ignored.",
            ),
            id="lint-ignored",
        ),
        pytest.param(
            False,
            True,
            LinterIssue(
                name="metadata",
                result=LinterResult.IGNORED,
                text="Metadata field 'source-code' is ignored.",
            ),
            id="skip-ignored",
        ),
    ],
)
def test_metadata_linter_source_code(
    new_dir: Path,
    snap_yaml_data: Callable[..., dict[str, Any]],
    lint_source: bool,
    specify_source: bool,
    linter_issue: LinterIssue | None,
) -> None:
    """Make sure that the source-code field is handled properly.

    https://github.com/canonical/snapcraft/issues/5794"""
    yaml_data = snap_yaml_data()
    if specify_source:
        yaml_data["source-code"] = "https://github.com/torvalds/linux"
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir,
        lint=models.Lint(ignore=[] if lint_source else [{"metadata": ["source-code"]}]),
    )

    if linter_issue is None:
        assert "source-code" not in str(issues)
    else:
        assert linter_issue in issues


def test_metadata_linter_ignore_devel_grade(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(**{"grade": "devel"})
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert len(issues) == 0
