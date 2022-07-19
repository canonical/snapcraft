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

import pytest

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
