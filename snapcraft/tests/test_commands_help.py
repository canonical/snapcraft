# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
from unittest import mock

import fixtures

from snapcraft import tests
from snapcraft.commands import help


class HelpCommandTestCase(tests.TestCase):

    def test_topic_and_plugin_not_found_exits_with_tip(self):
        fake_logger = fixtures.FakeLogger()
        self.useFixture(fake_logger)

        with self.assertRaises(SystemExit) as raised:
            help.main(['does-not-exist'])

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual('The plugin does not exist. Run `snapcraft '
                         'list-plugins` to see the available plugins.\n',
                         fake_logger.output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_module_help_when_no_help_for_valid_plugin(
            self, mock_stdout):
        help.main(['jdk'])

        self.assertEqual('The plugin has no documentation\n',
                         mock_stdout.getvalue())

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_module_help_for_valid_plugin(self, mock_stdout):
        help.main(['nil'])

        expected = 'The nil plugin is'
        output = mock_stdout.getvalue()[:len(expected)]
        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_show_module_help_with_devel_for_valid_plugin(self, mock_stdout):
        help.main(['nil', '--devel'])

        expected = 'Help on module snapcraft.plugins.nil in snapcraft.plugins'
        output = mock_stdout.getvalue()[:len(expected)]

        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_topics(self, mock_stdout):
        help.main(['topics'])

        output = mock_stdout.getvalue().strip().split('\n')
        for t in help._TOPICS:
            self.assertTrue(
                t in output, 'Missing topic: {!r} in {!r}'.format(t, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_topic_help_for_valid_topic(self, mock_stdout):
        help.main(['sources'])

        expected = 'Common keywords for plugins that use common source'
        output = mock_stdout.getvalue()[:len(expected)]
        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_topic_help_with_devel_for_valid_topic(self, mock_stdout):
        expected = {
            'sources': 'Help on module snapcraft',
            'plugins': 'Help on package snapcraft',
        }

        for key in help._TOPICS:
            mock_stdout.truncate(0)
            mock_stdout.seek(0)
            with self.subTest(key=key):
                help.main([key, '--devel'])
                output = mock_stdout.getvalue()[:len(expected[key])]
                self.assertEqual(
                    output, expected[key],
                    'The help message does not start with {!r} but with '
                    '{!r} instead'.format(expected[key], output))
