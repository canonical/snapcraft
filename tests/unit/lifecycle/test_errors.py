# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from snapcraft.internal.lifecycle import errors


class TestErrorFormatting:

    scenarios = (
        (
            "PackVerificationError",
            dict(
                exception_class=errors.PackVerificationError,
                kwargs={},
                expected_message="Failed to verify directory to pack.",
            ),
        ),
    )

    def test_error_formatting(self, exception_class, kwargs, expected_message):
        assert str(exception_class(**kwargs)) == expected_message
