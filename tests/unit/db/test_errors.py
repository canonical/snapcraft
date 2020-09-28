# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from snapcraft.internal.db import errors


def test_SnapcraftDatastoreVersionUnsupported():
    error = errors.SnapcraftDatastoreVersionUnsupported(
        path=pathlib.Path("/tmp/foo.yaml"), current_version=5, supported_version=4
    )

    assert (
        error.get_brief()
        == "This version of snapcraft does not support version 5 of the /tmp/foo.yaml datastore."
    )
    assert error.get_details() is None
    assert (
        error.get_resolution()
        == "Use 'snap revert' or 'snap refresh' to install previously used version of snapcraft (or newer)."
    )
    assert error.get_docs_url() is None
    assert error.get_reportable() is False
    assert error.get_exit_code() == 2
