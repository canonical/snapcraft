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
import pydoc
from unittest import mock

from snapcraft._help import _TOPICS
from snapcraft.main import main

from snapcraft import tests


class HelpCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        # pydoc pager guess can fail, for tests we want a plain pager
        # anyway
        p = mock.patch('pydoc.pager', new=pydoc.plainpager)
        p.start()
        self.addCleanup(p.stop)

    def test_topic_and_plugin_not_found_exits_with_tip(self):
        with self.assertRaises(SystemExit) as raised:
            main(['help', 'does-not-exist'])

        self.assertEqual('The plugin does not exist. Run `snapcraft '
                         'list-plugins` to see the\navailable plugins.',
                         str(raised.exception))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_module_help_when_no_help_for_valid_plugin(
            self, mock_stdout):
        main(['help', 'jdk'])

        self.assertEqual('The plugin has no documentation\n',
                         mock_stdout.getvalue())

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_module_help_for_valid_plugin(self, mock_stdout):
        main(['help', 'nil'])

        expected = 'The nil plugin is'
        output = mock_stdout.getvalue()[:len(expected)]
        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_show_module_help_with_devel_for_valid_plugin(self, mock_stdout):
        main(['help', 'nil', '--devel'])

        expected = 'Help on module snapcraft.plugins.nil in snapcraft.plugins'
        output = mock_stdout.getvalue()[:len(expected)]

        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_topics(self, mock_stdout):
        main(['help', 'topics'])

        output = mock_stdout.getvalue().strip().split('\n')
        for t in _TOPICS:
            self.assertTrue(
                t in output, 'Missing topic: {!r} in {!r}'.format(t, output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_print_topic_help_for_valid_topic(self, mock_stdout):
        main(['help', 'sources'])

        expected = "Common 'source' options."
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

        for key in _TOPICS:
            mock_stdout.truncate(0)
            mock_stdout.seek(0)
            with self.subTest(key=key):
                main(['help', key, '--devel'])
                output = mock_stdout.getvalue()[:len(expected[key])]
                self.assertEqual(
                    output, expected[key],
                    'The help message does not start with {!r} but with '
                    '{!r} instead'.format(expected[key], output))

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_no_unicode_in_help_strings(self, mock_stdout):
        helps = ['topics']

        for key in _TOPICS.keys():
            helps.append(str(key))

        # Get a list of plugins
        import snapcraft.plugins
        import os
        from pathlib import Path
        for plugin in Path(snapcraft.plugins.__path__[0]).glob('*.py'):
            if (os.path.isfile(str(plugin)) and
                    not os.path.basename(str(plugin)).startswith('_')):
                helps.append(os.path.basename(str(plugin)[:-3]))

        for key in helps:
            mock_stdout.truncate(0)
            mock_stdout.seek(0)
            with self.subTest(key=key):
                main(['help', key])
                try:
                    mock_stdout.getvalue().encode('ascii')
                except UnicodeEncodeError:
                    self.fail('Non-ASCII characters in help text for '
                              '{!r}'.format(key))
