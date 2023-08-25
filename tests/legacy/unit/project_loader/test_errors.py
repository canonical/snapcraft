# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020, 2023 Canonical Ltd
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

import pathlib

from snapcraft_legacy.internal.project_loader import errors


def test_SnapcraftProjectUnusedKeyAssetError():
    path = pathlib.Path("foo")

    error = errors.SnapcraftProjectUnusedKeyAssetError(key_path=path)

    assert error.get_brief() == "Found unused key asset 'foo'."
    assert error.get_details() == "All configured key assets must be utilized."
    assert error.get_resolution() == "Verify key usage and remove all unused keys."


def test_variable_evaluation_error():
    error = errors.VariableEvaluationError(
        variable="TEST_VARIABLE", reason="reason", docs_url="www.example.com"
    )

    assert str(error) == (
        "Cannot evaluate project variable 'TEST_VARIABLE': reason\n"
        "For more information, check out: www.example.com"
    )
