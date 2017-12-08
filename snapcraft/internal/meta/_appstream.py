# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

from snapcraft.internal.meta import _errors

from xml.etree.ElementTree import (
    ElementTree,
    ParseError
)


class AppstreamInfoParser:

    def __init__(self, path):
        self._path = path

    def extract(self, keys):
        info = {}
        try:
            tree = ElementTree().parse(self._path)
        except ParseError:
            raise _errors.AppstreamFileParseError(self._path)
        for key in keys:
            child = tree.find(key)
            if child is not None:
                info[key] = child.text
        return info
