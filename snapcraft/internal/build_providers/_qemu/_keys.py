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

import os
from typing import TypeVar, Type

from paramiko import RSAKey

from snapcraft.internal.errors import SnapcraftError


SSHKeyT = TypeVar('SSHKeyT', bound='SSHKey')


class SSHKeyPathError(SnapcraftError):

    fmt = (
        '{private_key_file_path!r} does not exist. '
        'A private key is required.\n'
        'Please file a report on https://launchpad.net/snapcraft/+filebug'
    )

    def __init__(self, *, private_key_file_path: str) -> None:
        super().__init__(private_key_file_path=private_key_file_path)


class SSHKey:

    @classmethod
    def new_key(cls: Type[SSHKeyT], *, root_dir: str) -> SSHKeyT:
        private_key_file_path = os.path.join(root_dir, 'id_rsa')

        # Keep the amount of bits up to date with latest trends.
        key = RSAKey.generate(bits=4096)
        os.makedirs(os.path.dirname(private_key_file_path), exist_ok=True)
        key.write_private_key_file(private_key_file_path)
        return cls(root_dir=root_dir)

    def __init__(self, *, root_dir: str) -> None:
        private_key_file_path = os.path.join(root_dir, 'id_rsa')
        if not os.path.exists(private_key_file_path):
            raise SSHKeyPathError(private_key_file_path=private_key_file_path)

        self._key = RSAKey.from_private_key_file(private_key_file_path)
        self.private_key_file_path = private_key_file_path

    def get_public_key(self) -> str:
        return '{} {}'.format(self._key.get_name(), self._key.get_base64())
