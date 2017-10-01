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

from unittest import mock

import fixtures
from testtools.matchers import Equals

from snapcraft import tests, _store


class EditCollaboratorsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        patcher = mock.patch('subprocess.check_call')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_collaborators_must_write_header_and_developers(self):
        developers_from_assertion = [
            {'developer-id': 'test-dev-id1',
             'since': '2017-02-10T08:35:00.000000Z',
             'until': '2018-02-10T08:35:00.000000Z'},
            {'developer-id': 'test-dev-id2',
             'since': '2016-02-10T08:35:00.000000Z',
             'until': '2019-02-10T08:35:00.000000Z'},
        ]

        existing_developers = ("developers:\n"
                               "- developer-id: test-dev-id1\n"
                               "  since: '2017-02-10 08:35:00'\n"
                               "  until: '2018-02-10 08:35:00'\n"
                               "- developer-id: test-dev-id2\n"
                               "  since: '2016-02-10 08:35:00'\n"
                               "  until: '2019-02-10 08:35:00'\n")
        expected_written = (_store._COLLABORATION_HEADER + '\n' +
                            existing_developers)

        with mock.patch(
                'builtins.open',
                new_callable=mock.mock_open,
                read_data=expected_written) as mock_open:
            developers_for_assertion = _store._edit_collaborators(
                developers_from_assertion)

        written = ''
        for call in mock_open().write.call_args_list:
            written += str(call.call_list()[0][0][0])
        self.assertThat(written, Equals(expected_written))
        self.assertThat(
            developers_for_assertion, Equals(developers_from_assertion))

    def test_edit_collaborators_must_return_new_developers(self):
        developers_from_assertion = [
            {'developer-id': 'test-dev-id1',
             'since': '2017-02-10T08:35:00.000000Z',
             'until': '2018-02-10T08:35:00.000000Z'},
        ]

        new_developers = ("developers:\n"
                          "- developer-id: test-dev-id1\n"
                          "  since: '2017-02-10 08:35:00'\n"
                          "  until: '2018-02-10 08:35:00'\n"
                          "- developer-id: test-dev-id2\n"
                          "  since: '2016-02-10 08:35:00'\n"
                          "  until: '2019-02-10 08:35:00'\n")

        with mock.patch('builtins.open', new_callable=mock.mock_open,
                        read_data=new_developers):
            developers_for_assertion = _store._edit_collaborators(
                developers_from_assertion)

        expected_developers = developers_from_assertion + [{
            'developer-id': 'test-dev-id2',
            'since': '2016-02-10T08:35:00.000000Z',
            'until': '2019-02-10T08:35:00.000000Z',
        }]
        self.assertThat(developers_for_assertion, Equals(expected_developers))


class EditCollaboratorsOpenEditorTestCase(tests.TestCase):

    scenarios = (
        ('default', {'editor': None, 'expected': 'vi'}),
        ('non-default', {'editor': 'test-editor', 'expected': 'test-editor'})
    )

    def setUp(self):
        super().setUp()
        patcher = mock.patch('subprocess.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_collaborators_must_open_editor(self):
        self.useFixture(fixtures.EnvironmentVariable('EDITOR', self.editor))
        _store._edit_collaborators({})
        self.check_call_mock.assert_called_with([self.expected, mock.ANY])
