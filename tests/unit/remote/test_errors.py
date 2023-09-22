# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

from snapcraft.remote import errors


def test_git_error():
    """Test GitError."""
    error = errors.GitError("Error details.")

    assert str(error) == "Git operation failed.\nError details."
    assert (
        repr(error)
        == "GitError(brief='Git operation failed.', details='Error details.')"
    )
    assert error.brief == "Git operation failed."
    assert error.details == "Error details."
