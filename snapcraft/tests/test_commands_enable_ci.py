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

import logging
from unittest import mock

import fixtures

from snapcraft import tests
from snapcraft.integrations import travis
from snapcraft.main import main


class EnableCITestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def test_enable_ci_empty(self):
        with self.assertRaises(SystemExit) as raised:
            main(['enable-ci'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual([
            'Please select one of the supported integration systems: travis.'
        ], self.fake_logger.output.splitlines())

    def test_enable_ci_unknown(self):
        with self.assertRaises(SystemExit) as raised:
            main(['enable-ci', 'bazinga'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual([
            '"bazinga" integration is not supported by snapcraft.',
            'Please select one of the supported integration systems: travis.'
        ], self.fake_logger.output.splitlines())

    @mock.patch.object(travis, 'enable')
    @mock.patch('builtins.input')
    def test_enable_ci_travis(self, mock_input, mock_enable):
        mock_input.side_effect = ['y']
        main(['enable-ci', 'travis'])

        self.assertEqual(1, mock_enable.call_count)
        self.assertEqual([
            'Snapcraft integration for Travis (CI).',
            '',
            'This is an *EXPERIMENTAL* feature and subject to incompatible '
            'changes in',
            'the future, please use with caution.',
            '',
            'This command currently depends on a working `travis` CLI '
            'environment and',
            'a previously initialized Travis project (`.travis.yml`).',
            '',
            'Make sure your Travis project is also configured to '
            '"Build pushes", this',
            'way every new push to `master` will result in a new snap '
            'revision in the',
            'Store.',
            '',
            'This operation will acquire properly attenuated Store '
            'credentials and',
            'encrypt them for use in your testbed '
            '(`.snapcraft/travis_snapcraft.cfg`),',
            'only Travis has the private key to decrypt it and will '
            'be only available',
            'to branches of the same repository, not forks.',
            '',
            "Then it will adjust Travis configuration ('.travis.yml') with "
            'the commands',
            "to decrypt credentials during 'after_success' phase and install "
            'latest',
            '`snapcraft` to build and release your snap (inside a '
            'ubuntu:xenial docker',
            "container) during the 'deploy' phase.",
            '',
            'See the example below::',
            '',
            '    sudo: required',
            '    services:',
            '    - docker',
            '    after_success:',
            '    - openssl aes-256-cbc -K <travis-key> -iv <travis-iv>',
            '      -in .snapcraft/travis_snapcraft.cfg',
            '      -out .snapcraft/snapcraft.cfg -d',
            '    deploy:',
            '      skip_cleanup: true',
            '      provider: script',
            '      script: docker run -v $(pwd):$(pwd) -t ubuntu:xenial sh -c',
            '        "apt update -qq && apt install snapcraft -y && '
            'cd $(pwd) &&',
            '        snapcraft && snapcraft push *.snap --release edge"',
            '      on:',
            '        branch: master',
            ''
        ], self.fake_terminal.getvalue().splitlines())
