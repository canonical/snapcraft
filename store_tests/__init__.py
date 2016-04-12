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
"""Tests against the store.

Unlike the integration tests, these tests don't use the snapcraft executable
(FIXME: almost) so they can be used to debug interactively.

To achieve isolation, they rely on each user providing credentials via
TEST_USER_EMAIL and TEST_USER_PASSWORD environment variables (unlike
integration tests that default to u1test+snapcraft@canonical.com).

As such they are expected to be run locally until proper isolation is achieved
for registered names on the staging server.
"""

import fileinput
import logging
import os
import shutil
import subprocess
import uuid

import fixtures
import testtools
from testtools import content

from snapcraft import (
    config,
    storeapi,
)
from snapcraft.tests import fixture_setup


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        # Run snapcraft from sources
        self.snapcraft_command = os.path.join(os.getcwd(), 'bin', 'snapcraft')
        # Keep track of the snaps used as a base by tests
        self.snaps_dir = os.path.join(
            os.path.dirname(__file__), '..', 'integration_tests', 'snaps')

        # Always work in a temp dir cleaned up at the end of the test
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        # Use a test-local config
        self.useFixture(fixtures.EnvironmentVariable(
            'XDG_CONFIG_HOME', os.path.join(self.path, '.config')))

        # Default to the staging environment
        self.useFixture(fixture_setup.StagingStore())

        # Capture logging
        self.logger = fixtures.LoggerFixture(level=logging.INFO)
        self.useFixture(self.logger)
        # INFO from the requests lib is too noisy
        logging.getLogger("requests").setLevel(logging.WARNING)

    def login(self, email=None, password=None):
        email = email or os.getenv('TEST_USER_EMAIL',
                                   'u1test+snapcraft@canonical.com')
        password = password or os.getenv('TEST_USER_PASSWORD', None)
        if not password:
            self.skipTest('No password provided for the test user.')

        # FIXME: Find a way to support one-time-passwords (otp)
        # -- vila 2016-04-11
        resp = storeapi.login(email, password, token_name='snapcraft', otp='')
        if resp['success']:
            config.save_config(resp['body'])
        return resp

    def logout(self):
        # Our setup guarantee we'll clear the expected config file
        config.clear_config()

    # FIXME: This was copied from integration_tests and need to be refactored
    # to be properly shared. Roughly, we need a way to create a snap on the fly
    # from a minimal template given a registered name and a version. If there
    # is a way to avoid calling out 'snapcraft' itself but use the internal API
    # instead, even better.  -- vila 2016-04-12
    def run_snapcraft(self, command, project_dir=None):
        if isinstance(command, str):
            command = [command]
        if project_dir:
            if not os.path.exists(project_dir):
                cwd = self.copy_project_to_tmp(project_dir)
            else:
                cwd = os.path.join(self.path, project_dir)
        else:
            cwd = None
        try:
            return subprocess.check_output(
                [self.snapcraft_command, '--debug'] + command, cwd=cwd,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

    def copy_project_to_tmp(self, project_dir):
        tmp_project_dir = os.path.join(self.path, project_dir)
        shutil.copytree(
            os.path.join(self.snaps_dir, project_dir), tmp_project_dir)
        return tmp_project_dir

    def _update_version(self, project_dir, version=None):
        # Change to a random version.
        # The maximum size is 32 chars.
        if version is None:
            version = str(uuid.uuid4().int)[:32]
        updated_project_dir = self.copy_project_to_tmp(project_dir)
        yaml_file = os.path.join(project_dir, 'snapcraft.yaml')
        for line in fileinput.input(yaml_file, inplace=True):
            if 'version: ' in line:
                print('version: ' + version)
            else:
                print(line)
        return updated_project_dir

    def upload(self, snap_filename, snap_name):
        conf = config.load_config()
        return storeapi.upload(snap_filename, snap_name, config=conf)
