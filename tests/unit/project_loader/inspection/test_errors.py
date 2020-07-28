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

from snapcraft.internal.project_loader.inspection import errors


def test_SnapcraftNoSuchFileError():
    exception = errors.SnapcraftNoSuchFileError(path="some-path")
    assert (
        exception.get_brief()
        == "Failed to find part that provided path 'some-path': file does not exist."
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Check the file path and try again."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
    assert exception.get_exit_code() == 2


def test_SnapcraftInspectError():
    exception = errors.SnapcraftInspectError()
    assert exception.get_exit_code() == 3


def test_SnapcraftNoStepsRunError():
    exception = errors.SnapcraftNoStepsRunError()
    assert exception.get_brief() == "Failed to get latest step: no steps have run"
    assert exception.get_details() is None
    assert exception.get_resolution() == "Run 'snapcraft clean' and retry build."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
    assert exception.get_exit_code() == 3


def test_SnapcraftProvidesInvalidFilePathError():
    exception = errors.SnapcraftProvidesInvalidFilePathError(path="some-path")
    assert (
        exception.get_brief()
        == "Failed to find part that provided path 'some-path': file is not in the staging or priming area."
    )
    assert exception.get_details() is None
    assert (
        exception.get_resolution()
        == "Ensure the path is in the staging or priming area and try again."
    )
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
    assert exception.get_exit_code() == 3


def test_SnapcraftUntrackedFileError():
    exception = errors.SnapcraftUntrackedFileError(path="some-path")
    assert (
        exception.get_brief()
        == "Failed to find part that provided path 'some-path': it may have been provided by a scriplet."
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Run 'snapcraft clean' and retry build."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
    assert exception.get_exit_code() == 3
