# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Publish your app for Linux users for desktop, cloud, and IoT."""

import os

import pkg_resources  # type: ignore


def _get_version():
    if os.environ.get("SNAP_NAME") == "snapcraft":
        return os.environ["SNAP_VERSION"]
    try:
        return pkg_resources.require("snapcraft")[0].version
    except pkg_resources.DistributionNotFound:
        return "devel"


__version__ = _get_version()
