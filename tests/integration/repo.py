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

import contextlib
from urllib import parse

import apt
import requests_unixsocket
from requests import exceptions


def is_snap_installed(snap):
    return get_local_snap_info(snap) is not None


def get_local_snap_info(snap):
    local_snap_info = None
    with contextlib.suppress(exceptions.HTTPError):
        local_snap_info = _get_local_snap_info(_get_parsed_snap(snap)[0])
    return local_snap_info


def _get_parsed_snap(snap):
    if "/" in snap:
        sep_index = snap.find("/")
        snap_name = snap[:sep_index]
        snap_channel = snap[sep_index + 1 :]
    else:
        snap_name = snap
        snap_channel = ""
    return snap_name, snap_channel


def _get_snapd_socket_path_template():
    return "http+unix://%2Frun%2Fsnapd.socket/v2/{}"


def _get_local_snap_info(name):
    slug = "snaps/{}".format(parse.quote(name, safe=""))
    url = _get_snapd_socket_path_template().format(slug)
    with requests_unixsocket.Session() as session:
        snap_info = session.get(url)
    snap_info.raise_for_status()
    return snap_info.json()["result"]


def is_package_installed(name):
    with apt.Cache() as apt_cache:
        return apt_cache[name].installed
