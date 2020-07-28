# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018,2020 Canonical Ltd
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


from snapcraft.plugins.v1._python import errors


class TestErrorFormatting:

    scenarios = (
        (
            "PipListInvalidLegacyFormatError",
            {
                "exception_class": errors.PipListInvalidLegacyFormatError,
                "kwargs": {"output": "test-output"},
                "expected_message": (
                    "Failed to parse Python package list: "
                    "The returned output is not in the expected format:\n"
                    "test-output"
                ),
            },
        ),
    )

    def test_error_formatting(self, exception_class, kwargs, expected_message):
        assert str(exception_class(**kwargs)) == expected_message
