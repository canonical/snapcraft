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

from testtools.matchers import Equals, FileExists, FileContains, StartsWith, EndsWith

from tests import unit
from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._qemu._keys import SSHKey


class SSHKeyTest(unit.TestCase):
    def test_new_key(self):
        ssh_key = SSHKey.new_key(root_dir=self.path)

        self.assertThat("id_rsa", FileExists())
        self.assertThat(
            ssh_key.private_key_file_path, Equals(os.path.join(self.path, "id_rsa"))
        )
        self.assertThat(
            "id_rsa",
            FileContains(matcher=StartsWith("-----BEGIN RSA PRIVATE KEY-----")),
        )
        self.assertThat(
            "id_rsa", FileContains(matcher=EndsWith("-----END RSA PRIVATE KEY-----\n"))
        )

    def test_load_existing(self):
        # We need to first create a key
        SSHKey.new_key(root_dir=self.path)

        ssh_key = SSHKey(root_dir=self.path)

        self.assertThat(
            ssh_key.private_key_file_path, Equals(os.path.join(self.path, "id_rsa"))
        )

    def test_get_public_key(self):
        ssh_key = SSHKey.new_key(root_dir=self.path)

        self.assertThat(ssh_key.get_public_key(), StartsWith("ssh-rsa "))

    def test_init_with_no_id_rsa(self):
        self.assertRaises(errors.SSHKeyFileNotFoundError, SSHKey, root_dir=self.path)
