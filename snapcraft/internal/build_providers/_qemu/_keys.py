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

from snapcraft.internal.build_providers import errors

SSHKeyT = TypeVar("SSHKeyT", bound="SSHKey")


class SSHKey:
    """SSHKey provides primitives to create and use RSA keys with SSH.

    The general use case is that if an instance of SSHKey returns
    SSHKeyPathFileNotFoundError new_key can be called.

    :ivar str private_key_file_path: the absolute path to the private key.
    """

    @classmethod
    def new_key(cls: Type[SSHKeyT], *, root_dir: str) -> SSHKeyT:
        """Create a new RSA key and return an instance of SSHKey.

        :param str rootdir: the path to the directory to store the private key.
        :returns: an instance of SSHKey using the newly generated key.
        :rtype: SSHKey
        """
        private_key_file_path = os.path.join(root_dir, "id_rsa")

        # Keep the amount of bits up to date with latest trends.
        key = RSAKey.generate(bits=4096)
        os.makedirs(os.path.dirname(private_key_file_path), exist_ok=True)
        key.write_private_key_file(private_key_file_path)
        return cls(root_dir=root_dir)

    def __init__(self, *, root_dir: str) -> None:
        """Instantiate an SSHKey with the RSA key stored in root_dir.

        :param str root_dir: the path to the directory where the private key
                             is stored.
        :raises SSHKeyPathFileNotFoundError:
            raised when the private key cannot be found. This exception should
            generally be handled by the use of new_key.
        """
        private_key_file_path = os.path.join(root_dir, "id_rsa")
        if not os.path.exists(private_key_file_path):
            raise errors.SSHKeyFileNotFoundError(
                private_key_file_path=private_key_file_path
            )

        self._key = RSAKey.from_private_key_file(private_key_file_path)
        self.private_key_file_path = private_key_file_path

    def get_public_key(self) -> str:
        """Return the public key formatted for use as an authorized keys entry.

        The returned string can be used as an entry for cloud init's
        ssh_authorized_keys or ssh's authorized_keys file.

        :returns: the public key formatted as 'ssh-rsa <hash>'.
        :rtype: str.
        """
        return "{} {}".format(self._key.get_name(), self._key.get_base64())
