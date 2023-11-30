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
import textwrap

import pytest
import yaml

from snapcraft import models
from snapcraft.linters.base import LinterResult


class TestLinterResult:
    """Linter result presentation."""

    def test_linter_result(self):
        assert f"{LinterResult.OK}" == "ok"
        assert f"{LinterResult.WARNING}" == "warning"
        assert f"{LinterResult.ERROR}" == "error"
        assert f"{LinterResult.FATAL}" == "fatal"
        assert f"{LinterResult.IGNORED}" == "ignored"


class TestLinterIssue:
    """Linter issue creation and presentation."""

    @pytest.mark.parametrize("result", list(LinterResult))
    def test_linter_issue(self, linter_issue, result):
        issue = linter_issue(result=result, filename="foo.txt")
        assert issue.name == "test"
        assert issue.filename == "foo.txt"
        assert issue.text == "Linter message text"
        assert issue.result == result
        assert issue.url == "https://some/url"

    def test_linter_issue_string(self, linter_issue):
        issue = linter_issue()
        assert f"{issue}" == "test: Linter message text (https://some/url)"

    def test_linter_issue_string_filename(self, linter_issue):
        issue = linter_issue(filename="foo.txt")
        assert f"{issue}" == "test: foo.txt: Linter message text (https://some/url)"

    def test_linter_issue_string_no_url(self, linter_issue):
        issue = linter_issue(url=None)
        assert f"{issue}" == "test: Linter message text"

    def test_linter_issue_string_filename_no_url(self, linter_issue):
        issue = linter_issue(filename="foo.txt", url=None)
        assert f"{issue}" == "test: foo.txt: Linter message text"


@pytest.fixture
def lint_ignore_data():
    """Yaml-loaded test data for specification of ignoring linter issues.

    The data specifies that all issues for linter "linter1" should be ignored, and that the
    issues for "linter2" should be ignored if they refer to files matching "file1" and/or
    "/lib/file2*".
    """
    yaml_data = textwrap.dedent(
        """
        ignore:
          - linter1           # ignore all issues for "linter1"
          - linter2:          # ignore issues on "file1" and "/lib/file2*" for "linter2"
            - "file1"
            - "/lib/file2*"
        """
    )
    return yaml.safe_load(yaml_data)


def test_lint_all_ignored(lint_ignore_data):
    lint = models.Lint(**lint_ignore_data)

    assert lint.all_ignored("linter1")
    assert not lint.all_ignored("linter2")
    assert not lint.all_ignored("linter3")


def test_lint_ignored_files(lint_ignore_data):
    lint = models.Lint(**lint_ignore_data)

    assert lint.ignored_files("linter1") == ["*"]
    assert lint.ignored_files("linter2") == ["file1", "/lib/file2*"]
    assert lint.ignored_files("linter3") == []
