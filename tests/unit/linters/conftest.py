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

from typing import Optional

import pytest

from snapcraft.linters.base import LinterIssue, LinterResult


@pytest.fixture
def linter_issue():
    def _create_issue(
        *,
        result: LinterResult = LinterResult.OK,
        filename: Optional[str] = None,
        text: str = "Linter message text"
    ):
        return LinterIssue(
            name="test",
            result=result,
            filename=filename,
            text=text,
            url="https://some/url",
        )

    yield _create_issue
