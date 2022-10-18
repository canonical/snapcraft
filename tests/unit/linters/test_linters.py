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

from pathlib import Path
from typing import List
from unittest.mock import call

import pytest
from overrides import overrides

from snapcraft import linters, projects
from snapcraft.linters.base import Linter, LinterResult
from snapcraft.linters.linters import _ignore_matching_filenames
from snapcraft.meta import snap_yaml


class TestLinterReport:
    """Linter report output."""

    def test_linter_report(self, emitter, linter_issue):
        issues = [
            linter_issue(result=LinterResult.WARNING, text="Without filename"),
            linter_issue(filename="foo.txt", text="With filename"),
            linter_issue(
                result=LinterResult.ERROR, filename="bar.txt", text="Some error"
            ),
        ]
        linters.report(issues)
        assert emitter.interactions == [
            call("message", "Lint OK:"),
            call("message", "- test: foo.txt: With filename (https://some/url)"),
            call("message", "Lint warnings:"),
            call("message", "- test: Without filename (https://some/url)"),
            call("message", "Lint errors:"),
            call("message", "- test: bar.txt: Some error (https://some/url)"),
        ]

    def test_linter_report_intermediate(self, emitter, linter_issue):
        issues = [
            linter_issue(result=LinterResult.WARNING, text="Without filename"),
            linter_issue(filename="foo.txt", text="With filename"),
            linter_issue(
                result=LinterResult.ERROR, filename="bar.txt", text="Some error"
            ),
        ]
        linters.report(issues, intermediate=True)
        assert emitter.interactions == [
            call("progress", "Lint OK:", permanent=True),
            call(
                "progress",
                "- test: foo.txt: With filename (https://some/url)",
                permanent=True,
            ),
            call("progress", "Lint warnings:", permanent=True),
            call(
                "progress",
                "- test: Without filename (https://some/url)",
                permanent=True,
            ),
            call("progress", "Lint errors:", permanent=True),
            call(
                "progress",
                "- test: bar.txt: Some error (https://some/url)",
                permanent=True,
            ),
        ]

    def test_linter_report_json(self, emitter, linter_issue):
        issues = [
            linter_issue(result=LinterResult.WARNING),
            linter_issue(filename="foo.txt"),
        ]
        linters.report(issues, json_output=True)
        assert emitter.interactions == [
            call(
                "message",
                '[{"type": "lint", "name": "test", "result": "warning", "text": '
                '"Linter message text", "url": "https://some/url"}, {"type": '
                '"lint", "name": "test", "result": "ok", "filename": "foo.txt", '
                '"text": "Linter message text", "url": "https://some/url"}]',
            )
        ]


class TestLinterStatus:
    """Check report status according to issues reported."""

    # Expected results:
    # 0: found no errors or warnings
    # 1: checks not run due to fatal error
    # 2: found only errors or errors and warnings
    # 3: found only warnings

    @pytest.mark.parametrize(
        "results,expected_status",
        [
            ([], 0),
            ([LinterResult.OK], 0),
            ([LinterResult.IGNORED], 0),
            ([LinterResult.WARNING], 3),
            ([LinterResult.ERROR], 2),
            ([LinterResult.FATAL], 1),
            ([LinterResult.OK, LinterResult.WARNING], 3),
            ([LinterResult.IGNORED, LinterResult.WARNING], 3),
            ([LinterResult.OK, LinterResult.WARNING, LinterResult.ERROR], 2),
            ([LinterResult.OK, LinterResult.ERROR, LinterResult.FATAL], 1),
        ],
    )
    def test_linter_status(self, linter_issue, results, expected_status):
        issues = []
        for result in results:
            issues.append(linter_issue(result=result))

        status = linters.report(issues)
        assert status == expected_status


class TestLinterRun:
    """Check linter execution."""

    class _TestLinter(Linter):
        @overrides
        def run(self) -> List[linters.LinterIssue]:
            assert self._snap_metadata.name == "mytest"
            return [
                linters.LinterIssue(
                    name="test",
                    result=LinterResult.WARNING,
                    text="Something wrong.",
                    url="https://some/url",
                )
            ]

    def test_run_linters(self, mocker, new_dir, linter_issue):
        mocker.patch(
            "snapcraft.linters.linters.LINTERS", {"test": TestLinterRun._TestLinter}
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
            linters.LinterIssue(
                name="test",
                result=LinterResult.WARNING,
                text="Something wrong.",
                url="https://some/url",
            )
        ]

    def test_run_linters_ignore(self, mocker, new_dir, linter_issue):
        mocker.patch(
            "snapcraft.linters.linters.LINTERS", {"test": TestLinterRun._TestLinter}
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

        lint = projects.Lint(ignore=["test"])
        issues = linters.run_linters(new_dir, lint=lint)
        assert issues == []

    def test_ignore_matching_filenames(self, linter_issue):
        lint = projects.Lint(ignore=[{"test": ["foo*", "some/dir/*"]}])
        issues = [
            linter_issue(filename="foo.txt", result=LinterResult.WARNING),
            linter_issue(filename="bar.txt", result=LinterResult.WARNING),
            linter_issue(filename="some/dir/baz.txt", result=LinterResult.ERROR),
            linter_issue(filename="other/dir/quux.txt", result=LinterResult.ERROR),
        ]

        _ignore_matching_filenames(issues, lint=lint)
        assert issues == [
            linter_issue(filename="foo.txt", result=LinterResult.IGNORED),
            linter_issue(filename="bar.txt", result=LinterResult.WARNING),
            linter_issue(filename="some/dir/baz.txt", result=LinterResult.IGNORED),
            linter_issue(filename="other/dir/quux.txt", result=LinterResult.ERROR),
        ]
