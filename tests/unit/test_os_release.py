# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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
from textwrap import dedent

import pytest

from snapcraft import errors, os_release


@pytest.fixture
def _os_release(new_dir):
    def _release_data(contents):
        path = Path("os-release")
        path.write_text(contents)
        release = os_release.OsRelease(os_release_file=path)
        return release

    return _release_data


def test_blank_lines(_os_release):
    release = _os_release(
        dedent(
            """\
            NAME="Arch Linux"

            PRETTY_NAME="Arch Linux"
            ID=arch
            ID_LIKE=archlinux
            VERSION_ID="foo"
            VERSION_CODENAME="bar"

            """
        )
    )

    assert release.id() == "arch"
    assert release.name() == "Arch Linux"
    assert release.version_id() == "foo"
    assert release.version_codename() == "bar"


def test_no_id(_os_release):
    release = _os_release(
        dedent(
            """\
            NAME="Arch Linux"
            PRETTY_NAME="Arch Linux"
            ID_LIKE=archlinux
            VERSION_ID="foo"
            VERSION_CODENAME="bar"
            """
        )
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        release.id()

    assert str(raised.value) == "Unable to determine host OS ID"


def test_no_name(_os_release):
    release = _os_release(
        dedent(
            """\
            ID=arch
            PRETTY_NAME="Arch Linux"
            ID_LIKE=archlinux
            VERSION_ID="foo"
            VERSION_CODENAME="bar"
            """
        )
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        release.name()

    assert str(raised.value) == "Unable to determine host OS name"


def test_no_version_id(_os_release):
    release = _os_release(
        dedent(
            """\
            NAME="Arch Linux"
            ID=arch
            PRETTY_NAME="Arch Linux"
            ID_LIKE=archlinux
            VERSION_CODENAME="bar"
            """
        )
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        release.version_id()

    assert str(raised.value) == "Unable to determine host OS version ID"


def test_no_version_codename(_os_release):
    """Test that version codename can also come from VERSION_ID"""
    release = _os_release(
        dedent(
            """\
            NAME="Ubuntu"
            VERSION="14.04.5 LTS, Trusty Tahr"
            ID=ubuntu
            ID_LIKE=debian
            PRETTY_NAME="Ubuntu 14.04.5 LTS"
            VERSION_ID="14.04"
            """
        )
    )

    assert release.version_codename() == "trusty"


def test_no_version_codename_or_version_id(_os_release):
    release = _os_release(
        dedent(
            """\
            NAME="Ubuntu"
            ID=ubuntu
            ID_LIKE=debian
            PRETTY_NAME="Ubuntu 16.04.3 LTS"
            """
        )
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        release.version_codename()

    assert str(raised.value) == "Unable to determine host OS version codename"
