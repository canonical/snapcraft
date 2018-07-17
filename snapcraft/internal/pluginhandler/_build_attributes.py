# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


class BuildAttributes:
    def __init__(self, build_attributes):
        self._attributes = build_attributes

    def no_system_libraries(self):
        return "no-system-libraries" in self._attributes

    def no_patchelf(self):
        return "no-patchelf" in self._attributes

    def keep_execstack(self):
        return "keep-execstack" in self._attributes
