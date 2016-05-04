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
import io
import os

import fixtures
import requests
import testtools

from snapcraft.tests import (
    fixture_setup,
    store_tests,
)


class TestTape(testtools.TestCase):

    def setUp(self):
        super().setUp()
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

    def test_recording_creates_tape(self):
        path = os.path.join(self.path, 'tapes', 'foo')
        self.assertFalse(os.path.exists(path))
        tape = store_tests.Tape(path, True)
        tape.close()
        self.assertTrue(os.path.exists(path))

    def test_record_utf8_content(self):
        path = os.path.join(self.path, 'tapes', 'foo')
        self.assertFalse(os.path.exists(path))
        tape = store_tests.Tape(path, True)
        response = requests.Response()
        response.__setstate__(dict(_content=b'[]'))
        tape.record(response)
        self.assertEqual(1, len(tape.records))
        record = tape.records[0]
        self.assertIn('content', record)

    def test_record_binary_content(self):
        path = os.path.join(self.path, 'tapes', 'foo')
        self.assertFalse(os.path.exists(path))
        tape = store_tests.Tape(path, True)
        response = requests.Response()
        response.__setstate__(dict(_content=b'\xff'))
        tape.record(response)
        self.assertEqual(1, len(tape.records))
        record = tape.records[0]
        self.assertNotIn('content', record)
        self.assertIn('b64_content', record)


class TestRecordedTest(testtools.TestCase):

    def test_test_without_tape_is_skipped(self):
        class TestWithoutTape(store_tests.RecordedTestCase):

            recording = False

            def test_me(self):
                pass
        out = io.StringIO()
        res = testtools.TextTestResult(out)
        test = TestWithoutTape('test_me')
        # We want to skip but not because the TEST_USER_PASSWORD is not set
        self.useFixture(fixtures.EnvironmentVariable('TEST_USER_PASSWORD',
                                                     'anything'))
        test.run(res)
        self.assertTrue(res.wasSuccessful())
        self.assertEqual(1, len(res.skip_reasons.values()))
        reason = list(res.skip_reasons.keys())[0]
        self.assertTrue(reason.startswith('No record available'))
