# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

from snapcraft.internal.repo.deb_package import DebPackage


def test_basic():
    package = DebPackage(name="foo", arch="arch", version="1.0")

    assert package.name == "foo"
    assert package.arch == "arch"
    assert package.version == "1.0"


def test_parse_simple():
    assert DebPackage.from_unparsed("foo") == DebPackage(
        name="foo", arch=None, version=None
    )


def test_parse_arch():
    assert DebPackage.from_unparsed("foo:arch") == DebPackage(
        name="foo", arch="arch", version=None
    )


def test_parse_arch_and_version():
    assert DebPackage.from_unparsed("foo:arch=4.5") == DebPackage(
        name="foo", arch="arch", version="4.5"
    )


def test_parse_version():
    assert DebPackage.from_unparsed("foo=4.5") == DebPackage(
        name="foo", arch=None, version="4.5"
    )
