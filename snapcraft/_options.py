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


class _Project:

    @property
    def use_geoip(self):
        return self.__use_geoip

    @use_geoip.setter
    def use_geoip(self, setting):
        self.__use_geoip = setting

    def __init__(self):
        self.__use_geoip = False


_project = _Project()


def get_project_options():
    return _project


def reset_project():
    global _project
    _project = _Project()
