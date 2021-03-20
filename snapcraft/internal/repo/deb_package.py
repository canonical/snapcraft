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

from typing import Optional

import attr


@attr.s(auto_attribs=True)
class DebPackage(object):
    """Debian package representation."""

    name: str
    arch: Optional[str] = None
    version: Optional[str] = None

    @classmethod
    def from_unparsed(cls, package: str) -> "DebPackage":
        """Parse package supported in yaml.

        Package Format:
        <package-name>[:<arch>][=<version>]

        Examples:
        "foo"
        "foo:i386"
        "foo=1.5"
        "foo:i386=1.5"

        :param package: Package to parse.

        :returns: DebPackage with populated arch & version, if any.
        """
        parsed_name: str = package
        parsed_arch: Optional[str] = None
        parsed_version: Optional[str] = None

        if "=" in parsed_name:
            parsed_name, parsed_version = parsed_name.split("=")

        if ":" in parsed_name:
            parsed_name, parsed_arch = parsed_name.split(":")

        return DebPackage(name=parsed_name, arch=parsed_arch, version=parsed_version)
