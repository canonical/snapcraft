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

import logging
import os
import subprocess
import sys
import uuid

import fixtures
import testtools

import snapcraft
from snapcraft import (
    common,
    config,
    lifecycle,
    storeapi,
)
from snapcraft.storeapi import _upload
from snapcraft.tests import fixture_setup


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        # Run snapcraft from sources
        self.snapcraft_command = os.path.join(os.getcwd(), 'bin', 'snapcraft')
        # FIXME: Urgh isolation! -- vila 2016-04-12
        common.set_schemadir(os.path.join(__file__,
                             '..', '..', 'schema'))
        # Where the snap templates are
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

    def create_snap(self, name, version=None):
        """Create a test snap from a template.

        :param name: The snap template name in integration_tests/snaps and the
            directory where it's stored in the test private file system.

        :param version: An optional version for the created snap

        :returns: The path where the binary snap file has been created.
        """
        if version is None:
            # Change to a random version. The maximum size is 32 chars.
            version = str(uuid.uuid4().int)[:32]
        template = '''
name: {name}
version: {version}
summary: Summary of the most simple test snap
description: Description of the most simple test snap

parts:
    part1:
      plugin: nil
'''
        snap_dir = os.path.join(self.path, name)
        os.mkdir(snap_dir)
        yaml_path = os.path.join(snap_dir, 'snapcraft.yaml')
        with open(yaml_path, 'w') as f:
            f.write(template.format(name=name, version=version))
        snap_path = os.path.join(self.path, 'snap.snap')
        real_check_call = subprocess.check_call

        def check_call(*args):
            # Swallow output as it's only relevant for debug
            return real_check_call(*args, stdout=subprocess.DEVNULL)
        self.addCleanup(
            setattr, subprocess, 'check_call', real_check_call)
        subprocess.check_call = check_call
        os.chdir(snap_dir)
        lifecycle.snap(snapcraft.ProjectOptions(), None, snap_path)
        return snap_path, name

    def register(self, snap_name):
        conf = config.load_config()
        res = storeapi.register_name(conf, snap_name)
        return res

    def upload(self, snap_filename, snap_name):
        conf = config.load_config()
        # Diable the progress indications, we don't need them during tests
        orig = _upload.ProgressBar

        class Silent(orig):

            def start(self):
                pass

            def update(self, value=None):
                pass

            def finish(self):
                pass

        try:
            _upload.ProgressBar = Silent
            res = storeapi.upload(snap_filename, snap_name, config=conf)
        finally:
            _upload.ProgressBar = orig
        return res
