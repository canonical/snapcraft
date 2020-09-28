# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from typing import Dict

_BASE_IMAGE = dict(core="16.04", core16="16.04", core18="18.04", core20="20.04")


def get_image_source(*, base: str) -> Dict[str, str]:
    """Return a valid source to build for base.

    :param str base: the base to target.
    :returns: a dictionary with a valid source for PyLXD.
    :raises KeyError: when a non supported base is used.
    """
    # TODO catch KeyError and raise an appropriate exception.
    return dict(
        type="image",
        mode="pull",
        server="https://cloud-images.ubuntu.com/buildd/releases",
        protocol="simplestreams",
        alias=_BASE_IMAGE[base],
    )
