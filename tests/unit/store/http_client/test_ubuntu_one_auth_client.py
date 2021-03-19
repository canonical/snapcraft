# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd
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

import pathlib

import pymacaroons
import pytest

from snapcraft.storeapi import http_clients


def test_invalid_macaroon_root_raises_exception(tmp_work_path):
    with pathlib.Path("conf").open("w") as config_fd:
        print("[login.ubuntu.com]", file=config_fd)
        print("macaroon=inval'id", file=config_fd)
        config_fd.flush()

    client = http_clients.UbuntuOneAuthClient()
    with pathlib.Path("conf").open() as config_fd:
        with pytest.raises(http_clients.errors.InvalidCredentialsError):
            client.login(config_fd=config_fd)


def test_invalid_discharge_raises_exception():
    with pathlib.Path("conf").open("w") as config_fd:
        print("[login.ubuntu.com]", file=config_fd)
        print("macaroon={}".format(pymacaroons.Macaroon().serialize()), file=config_fd)
        print("unbound_discharge=inval'id", file=config_fd)
        config_fd.flush()

    client = http_clients.UbuntuOneAuthClient()

    with pathlib.Path("conf").open() as config_fd:
        with pytest.raises(http_clients.errors.InvalidCredentialsError):
            client.login(config_fd=config_fd)
