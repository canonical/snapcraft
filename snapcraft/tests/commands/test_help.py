# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import logging
import pydoc
from unittest import mock

import fixtures
from testtools.matchers import Equals, StartsWith

from snapcraft.cli.help import _TOPICS

from . import CommandBaseTestCase


class HelpCommandBaseTestCase(CommandBaseTestCase):

    def setUp(self):
        super().setUp()
        # pydoc pager guess can fail, for tests we want a plain pager
        # anyway
        p = mock.patch('pydoc.pager', new=pydoc.plainpager)
        p.start()
        self.addCleanup(p.stop)


class HelpCommandTestCase(HelpCommandBaseTestCase):

    def test_topic_and_plugin_not_found_exits_with_tip(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        result = self.run_command(['help', 'does-not-exist'])

        self.assertEqual(1, result.exit_code)
        self.assertThat(
            'The plugin does not exist. Run `snapcraft '
            'list-plugins` to see the available plugins.\n',
            Equals(result.output))

    def test_print_module_help_when_no_help_for_valid_plugin(self):
        result = self.run_command(['help', 'jdk'])

        self.assertEqual('The plugin has no documentation\n', result.output)

    def test_print_module_help_for_valid_plugin(self):
        result = self.run_command(['help', 'nil'])

        expected = 'The nil plugin is'
        output = result.output[:len(expected)]
        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    def test_print_module_named_with_dashes_help_for_valid_plugin(self):
        result = self.run_command(['help', 'plainbox-provider'])

        expected = ' Create parts'
        self.assertThat(result.output, StartsWith(expected))

    def test_show_module_help_with_devel_for_valid_plugin(self):
        result = self.run_command(['help', 'nil', '--devel'])

        expected = 'Help on module snapcraft.plugins.nil in snapcraft.plugins'
        output = result.output[:len(expected)]

        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    def test_print_topics(self):
        result = self.run_command(['help', 'topics'])

        output = result.output.strip().split('\n')
        for t in _TOPICS:
            self.assertTrue(
                t in output, 'Missing topic: {!r} in {!r}'.format(t, output))

    def test_print_topic_help_for_valid_topic(self):
        result = self.run_command(['help', 'sources'])

        expected = "Common 'source' options."
        output = result.output[:len(expected)]
        self.assertEqual(output, expected,
                         'The help message does not start with {!r} but with '
                         '{!r} instead'.format(expected, output))

    def test_no_unicode_in_help_strings(self):
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
            result = self.run_command(['help', key])
            # An UnicodeEncodeError will be raised if the help text has
            # non-ASCII characters.
            result.output.encode('ascii')


class TopicWithDevelTestCase(HelpCommandBaseTestCase):

    scenarios = [(topic, dict(topic=topic)) for
                 topic in _TOPICS]

    def test_print_topic_help_with_devel_for_valid_topic(self):
        expected = {
            'sources': 'Help on package snapcraft',
            'plugins': 'Help on package snapcraft',
        }

        result = self.run_command(['help', self.topic, '--devel'])
        output = result.output[:len(expected[self.topic])]
        self.assertEqual(
            output, expected[self.topic],
            'The help message does not start with {!r} but with '
            '{!r} instead'.format(expected[self.topic], output))
