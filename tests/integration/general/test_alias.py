# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import stat

import yaml
from testtools.matchers import Equals, FileExists

from tests import integration


# FIXME: Ideally this would be a snaps test so we can run the aliases, but that
# requires snapd v2.20 to be available everywhere (it's not). We'll just make
# sure it builds instead.
class AliasTestCase(integration.TestCase):
    def test_alias(self):
        self.run_snapcraft("prime", "alias")
        file_path = os.path.join(self.prime_dir, "bin", "hello.sh")
        self.assertThat(file_path, FileExists())
        self.assertTrue(
            os.stat(file_path).st_mode & stat.S_IEXEC,
            "Expected hello.sh to be executable",
        )

        snap_yaml = os.path.join(self.prime_dir, "meta", "snap.yaml")
        self.assertThat(snap_yaml, FileExists())

        data = {}
        with open(snap_yaml) as fp:
            data = yaml.load(fp)

        expected_aliases = ["hi.sh", "howdy.sh"]
        self.assertThat(
            set(data["apps"]["hello"]["aliases"]), Equals(set(expected_aliases))
        )
