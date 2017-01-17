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

import os
import stat

import yaml

import integration_tests
from testtools.matchers import FileExists


# FIXME: Ideally this would be a snaps test so we can run the aliases, but that
# requires snapd v2.20 to be available everywhere (it's not). We'll just make
# sure it builds instead.
class AliasTestCase(integration_tests.TestCase):

    def test_alias(self):
        project_dir = 'alias'
        self.run_snapcraft('prime', project_dir)
        file_path = os.path.join(project_dir, 'prime', 'bin', 'hello.sh')
        self.assertThat(file_path, FileExists())
        self.assertTrue(os.stat(file_path).st_mode & stat.S_IEXEC,
                        'Expected hello.sh to be executable')

        snap_yaml = os.path.join(project_dir, 'prime', 'meta', 'snap.yaml')
        self.assertThat(
            snap_yaml,
            FileExists())

        data = {}
        with open(snap_yaml) as fp:
            data = yaml.load(fp)

        expected_aliases = ['hi.sh', 'howdy.sh']
        self.assertEqual(set(expected_aliases),
                         set(data['apps']['hello']['aliases']))
