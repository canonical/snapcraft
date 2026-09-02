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

from pathlib import Path
from unittest.mock import MagicMock, call

import pytest
from typing_extensions import override

from snapcraft import linters, models
from snapcraft.linters.base import Linter, LinterResult
from snapcraft.linters.linters import (
    LinterStatus,
    _ignore_matching_filenames,
    _update_status,
)
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


class _TestLinter(Linter):
    @override
    def run(self) -> list[linters.LinterIssue]:
        assert self._snap_metadata.name == "mytest"
        return [
            linters.LinterIssue(
                name="test",
                result=LinterResult.WARNING,
                text="Something wrong.",
                url="https://some/url",
            )
        ]

    @staticmethod
    def get_categories() -> list[str]:
        return ["test-1", "test-2"]

    def is_file_ignored(self, filepath: Path, category: str = "") -> bool:
        return self._is_file_ignored(filepath, category)


class TestLinterRun:
    """Check linter execution."""

    def test_run_linters(self, mocker, new_dir, linter_issue):
        mocker.patch("snapcraft.linters.linters.LINTERS", {"test": _TestLinter})
        yaml_data = {
            "name": "mytest",
            "version": "1.29.3",
            "base": "core22",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "parts": {},
        }

        project = models.Project.unmarshal(yaml_data)
        snap_yaml.write(
            project,
            prime_dir=Path(new_dir),
            arch="amd64",
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
        mocker.patch("snapcraft.linters.linters.LINTERS", {"test": _TestLinter})
        yaml_data = {
            "name": "mytest",
            "version": "1.29.3",
            "base": "core22",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "parts": {},
        }

        project = models.Project.unmarshal(yaml_data)
        snap_yaml.write(
            project,
            prime_dir=Path(new_dir),
            arch="amd64",
        )

        lint = models.Lint(ignore=["test"])
        issues = linters.run_linters(new_dir, lint=lint)
        assert issues == []

    def test_run_linters_ignore_all_categories(self, mocker, new_dir, linter_issue):
        """Verify that if the spec ignores all categories one-by-one, run_linters()
        exits early with no issues."""
        mocker.patch("snapcraft.linters.linters.LINTERS", {"test": _TestLinter})
        yaml_data = {
            "name": "mytest",
            "version": "1.29.3",
            "base": "core22",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "parts": {},
        }

        project = models.Project.unmarshal(yaml_data)
        snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

        lint = models.Lint(ignore=["test-1", "test-2"])
        issues = linters.run_linters(new_dir, lint=lint)
        assert issues == []

    def test_ignore_matching_filenames(self, linter_issue):
        lint = models.Lint(ignore=[{"test": ["foo*", "some/dir/*"]}])
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


def test_base_linter_is_file_ignored():
    """Test the base Linter class' ignore mechanism with categories."""
    lint = models.Lint(
        ignore=[
            {"test": ["test-path"]},
            {"test-1": ["test-1-path"]},
            {"test-2": ["test-2-path"]},
        ]
    )
    linter = _TestLinter(name="test", snap_metadata=MagicMock(), lint=lint)

    # The "test-path" Path must be ignored by the "main" filter and all categories.
    assert linter.is_file_ignored(Path("test-path"))
    assert linter.is_file_ignored(Path("test-path"), category="test-1")
    assert linter.is_file_ignored(Path("test-path"), category="test-2")

    # "test-1-path" is ignored by the "test-1" only
    assert not linter.is_file_ignored(Path("test-1-path"))
    assert linter.is_file_ignored(Path("test-1-path"), category="test-1")
    assert not linter.is_file_ignored(Path("test-1-path"), category="test-2")

    # Likewise for "test-2-path" is ignored by the "test-2" only
    assert not linter.is_file_ignored(Path("test-2-path"))
    assert not linter.is_file_ignored(Path("test-2-path"), category="test-1")
    assert linter.is_file_ignored(Path("test-2-path"), category="test-2")


class TestUpdateStatus:
    """Test the _update_status function to ensure severity can only increase, never decrease."""

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_fatal_always_wins(self, status: LinterStatus):
        """FATAL result should always set status to FATAL regardless of current status."""
        assert _update_status(status, LinterResult.FATAL) == LinterStatus.FATAL

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_error_upgrades_appropriately(self, status: LinterStatus):
        """ERROR result should upgrade status except when already FATAL."""
        if status == LinterStatus.FATAL:
            # Should NOT downgrade from FATAL
            assert _update_status(status, LinterResult.ERROR) == LinterStatus.FATAL
        else:
            # Should upgrade to ERRORS from any other status
            assert _update_status(status, LinterResult.ERROR) == LinterStatus.ERRORS

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_warning_upgrades_from_ok_and_info_only(
        self, status: LinterStatus
    ):
        """WARNING result should only upgrade from OK and INFO."""
        if status in (LinterStatus.OK, LinterStatus.INFO):
            assert _update_status(status, LinterResult.WARNING) == LinterStatus.WARNINGS
        else:
            # Should NOT downgrade from more severe statuses
            assert _update_status(status, LinterResult.WARNING) == status

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_info_upgrades_from_ok_only(self, status: LinterStatus):
        """INFO result should only upgrade from OK status."""
        if status == LinterStatus.OK:
            assert _update_status(status, LinterResult.INFO) == LinterStatus.INFO
        else:
            # Should NOT downgrade from any other status (this was the original bug!)
            assert _update_status(status, LinterResult.INFO) == status

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_ok_never_changes_status(self, status: LinterStatus):
        """OK result should never change the current status."""
        assert _update_status(status, LinterResult.OK) == status

    @pytest.mark.parametrize(("status"), LinterStatus)
    def test_update_status_ignored_never_changes_status(self, status: LinterStatus):
        """IGNORED result should never change the current status."""
        assert _update_status(status, LinterResult.IGNORED) == status

    def test_update_status_severity_progression(self):
        """Test that severity can progress through all levels but never regress."""
        # Start with OK
        status = LinterStatus.OK
        # OK -> INFO
        status = _update_status(status, LinterResult.INFO)
        assert status == LinterStatus.INFO
        # INFO -> WARNINGS (more INFO shouldn't change it)
        status = _update_status(status, LinterResult.INFO)
        assert status == LinterStatus.INFO
        status = _update_status(status, LinterResult.WARNING)
        assert status == LinterStatus.WARNINGS
        # WARNINGS -> ERRORS (more INFO/WARNING shouldn't change it)
        status = _update_status(status, LinterResult.INFO)
        assert status == LinterStatus.WARNINGS
        status = _update_status(status, LinterResult.WARNING)
        assert status == LinterStatus.WARNINGS
        status = _update_status(status, LinterResult.ERROR)
        assert status == LinterStatus.ERRORS
        # ERRORS -> FATAL (nothing else should change it)
        status = _update_status(status, LinterResult.INFO)
        assert status == LinterStatus.ERRORS
        status = _update_status(status, LinterResult.WARNING)
        assert status == LinterStatus.ERRORS
        status = _update_status(status, LinterResult.ERROR)
        assert status == LinterStatus.ERRORS
        status = _update_status(status, LinterResult.FATAL)
        assert status == LinterStatus.FATAL
        # FATAL should never change
        status = _update_status(status, LinterResult.OK)
        assert status == LinterStatus.FATAL
        status = _update_status(status, LinterResult.INFO)
        assert status == LinterStatus.FATAL
        status = _update_status(status, LinterResult.WARNING)
        assert status == LinterStatus.FATAL
        status = _update_status(status, LinterResult.ERROR)
        assert status == LinterStatus.FATAL

    def test_update_status_original_bug_case(self):
        """Test the specific bug case mentioned in the issue."""
        # Original bug: ERRORS + INFO incorrectly became INFO
        result = _update_status(LinterStatus.ERRORS, LinterResult.INFO)
        assert result == LinterStatus.ERRORS, (
            "Bug fix failed: ERRORS + INFO should stay ERRORS"
        )
        # Another case: WARNINGS + INFO should stay WARNINGS
        result = _update_status(LinterStatus.WARNINGS, LinterResult.INFO)
        assert result == LinterStatus.WARNINGS, "WARNINGS + INFO should stay WARNINGS"
        # FATAL + INFO should stay FATAL
        result = _update_status(LinterStatus.FATAL, LinterResult.INFO)
        assert result == LinterStatus.FATAL, "FATAL + INFO should stay FATAL"
