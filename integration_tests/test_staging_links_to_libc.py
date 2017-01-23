# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import subprocess

import integration_tests


class StageLibcLinksTestCase(integration_tests.TestCase):

    def test_staging_libc_links(self):
        project_dir = 'staging_links_to_libc'

        # First, stage libc6-dev via stage-packages
        self.run_snapcraft(['stage', 'from-package'], project_dir)

        # Now tar up the staging area
        subprocess.check_call(['tar', 'cf', 'stage.tar', 'stage/'])

        # Now attempt to stage the tarred staging area once again. This should
        # not conflict.
        try:
            self.run_snapcraft(['stage', 'from-tar'], project_dir)
        except subprocess.CalledProcessError as e:
            if 'have the following file paths in common' in e.output:
                self.fail('Parts unexpectedly conflicted')
            else:
                raise
