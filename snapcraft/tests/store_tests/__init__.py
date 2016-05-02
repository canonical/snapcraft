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
so they can be used to debug interactively.

To achieve isolation, they rely on each user providing credentials via
TEST_USER_EMAIL and TEST_USER_PASSWORD environment variables (unlike
integration tests that default to u1test+snapcraft@canonical.com).

As such they are expected to be run locally until proper isolation is achieved
for registered names on the staging server.
"""
import base64
import functools
import json
import logging
import os
import shutil
import subprocess
import uuid

import fixtures
import progressbar
import requests
import testtools

import snapcraft
from snapcraft import (
    storeapi,
)
from snapcraft.internal import (
    common,
    lifecycle,
)
from snapcraft.storeapi import _upload
from snapcraft.tests import (
    fixture_setup,
    test_config,
)

TAPES_DIR = os.path.join(os.path.dirname(__file__), 'tapes')


class SilentProgressBar(progressbar.ProgressBar):
    """A progress bar causing no spurious output during tests."""
    def start(self):
        pass

    def update(self, value=None):
        pass

    def finish(self):
        pass


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        # FIXME: Urgh isolation! -- vila 2016-04-12
        common.set_schemadir(os.path.join(snapcraft.__file__,
                             '..', '..', 'schema'))
        # Where the snap templates are
        self.snaps_dir = os.path.join(
            os.path.dirname(__file__), '..', 'integration_tests', 'snaps')

        # Always work in a temp dir cleaned up at the end of the test
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        # Use a test-local config
        test_config.isolate_for_config(self)

        # Default to the staging environment
        self.useFixture(fixture_setup.StagingStore())

        # Capture logging
        self.logger = fixtures.LoggerFixture(level=logging.INFO)
        self.useFixture(self.logger)
        # INFO from the requests lib is too noisy
        logging.getLogger("requests").setLevel(logging.WARNING)

        # Disable the progress indications, we don't need them during tests
        self.addCleanup(setattr, _upload, 'ProgressBar', _upload.ProgressBar)
        _upload.ProgressBar = SilentProgressBar

        self.store = storeapi.SCAClient()
        self.addCleanup(self.store.close)

    def login(self, email=None, password=None):
        email = email or os.getenv('TEST_USER_EMAIL',
                                   'u1test+snapcraft@canonical.com')
        env_password = os.getenv('TEST_USER_PASSWORD', None)
        password = password or env_password

        # FIXME: Find a way to test one-time-passwords (otp)
        # -- vila 2016-04-11
        return self.store.login(email, password, one_time_password='')

    def logout(self):
        return self.store.logout()

    def create_snap(self, name, version=None):
        """Create a test snap from a template.

        :param name: The snap template name in integration_tests/snaps and the
            directory where it's stored in the test private file system.

        :param version: An optional version for the created snap

        :returns: The path where the binary snap file has been created.
        """
        # CI envs may not provide the right packages
        if shutil.which('mksquashfs') is None:
            self.skipTest('mksquashfs (provided by squashfs-tools)'
                          ' is not installed')
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
        res = self.store.register_name(snap_name)
        return res

    def upload(self, snap_filename, snap_name):
        res = self.store.upload(snap_filename, snap_name)
        return res

    def download(self, snap_name, channel='stable', path='downloaded.snap'):
        return self.store.download(snap_name, channel, path)


class Tape(object):
    """A record of all requests issued during a given test.

    Several servers can be (and are) involved so the responses order is
    significant.

    This handles both recording the responses for all requests and replaying
    them afterwards. The same test can be used to record against the set of
    needed servers and can be replayed from there even if the servers are no
    more available.

    """

    def __init__(self, uniq, recording):
        self.path = os.path.join(TAPES_DIR, uniq)
        self.recording = recording
        # Responses are recorded in the received order so they can replayed in
        # the same order.
        self.records = []
        if not self.recording:
            # Load all the responses
            with open(self.path) as tape:
                self.records = json.loads(tape.read())

    def record(self, response):
        "Record the response for a given request."""
        # All requests expect the download ones can be encoded as
        # text. Downloads are binay and needs to be protected. 'replay' will
        # reflect that to reconstruct a proper response.
        content = None
        b64_content = None
        try:
            content = response.content.decode('utf8')
        except UnicodeDecodeError:
            b64_content = base64.b64encode(response.content).decode('utf8')
        recorded = dict(status_code=response.status_code,
                        encoding=response.encoding)
        if content is not None:
            recorded['content'] = content
        if b64_content is not None:
            recorded['b64_content'] = b64_content
        self.records.append(recorded)

    def replay(self):
        """Replay a response for a given request."""
        record = self.records.pop(0)
        response = requests.Response()
        decoded = dict(status_code=record['status_code'],
                       encoding=record['encoding'])
        if 'content' in record:
            decoded['_content'] = record['content'].encode('utf8')
        if 'b64_content' in record:
            decoded['_content'] = base64.b64decode(record['b64_content'])

        response.__setstate__(decoded)
        return response

    def close(self):
        if self.recording:
            # Record all the collected responses
            if not os.path.exists(TAPES_DIR):
                os.mkdir(TAPES_DIR)
            with open(self.path, 'w') as tape:
                tape.write(json.dumps(self.records, indent=4))


class SessionRecorder(requests.Session):
    """Record responses so they can be replayed without a real server."""

    def __init__(self, tape):
        super().__init__()
        self.tape = tape

    def request(self, method, url, *args, **kwargs):
        # This is the method used in the base class to implement GET, POST and
        # friends (we ony use GET and POST though).
        if self.tape.recording:
            response = super().request(method, url, *args, **kwargs)
            self.tape.record(response)
        else:
            response = self.tape.replay()
        return response


class RecordedTestCase(TestCase):
    """A test class recording all requests.

    The replayed tests require a run of the recording scenario or are skipped
    otherwise.
    """

    scenarios = (('record', dict(recording=True)),
                 ('replay', dict(recording=False)))

    def setUp(self):
        super().setUp()
        env_password = os.getenv('TEST_USER_PASSWORD', None)
        if self.recording and not env_password:
            self.skipTest('No password provided for the test user.')
        tape_path = '{}.{}.{}'.format(self.__class__.__module__,
                                      self.__class__.__name__,
                                      self._testMethodName)
        try:
            self.tape = Tape(tape_path, self.recording)
            self.addCleanup(self.tape.close)
        except FileNotFoundError:
            self.skip('No record available, run the recording tests first')
            self.addCleanup(self.tape.close)
        self.preserved_session_class = requests.Session
        self.addCleanup(setattr, requests, 'Session', requests.Session)
        requests.Session = functools.partial(SessionRecorder, self.tape)

        # The store created in the base class has not been used yet, replace it
        # so it takes the SessionRecorder into account
        self.store = storeapi.SCAClient()
        self.addCleanup(self.store.close)
