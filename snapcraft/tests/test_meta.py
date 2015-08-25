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

import os
from unittest.mock import (
    Mock,
    call,
    mock_open,
    patch,
)

from snapcraft import (
    common,
    meta,
    tests
)


class Compose(tests.TestCase):

    def setUp(self):
        self.orig_wrap_exe = meta._wrap_exe
        meta._wrap_exe = Mock(return_value='binary.wrapped')

        self.config_data = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
        }

    def tearDown(self):
        meta._wrap_exe = self.orig_wrap_exe

    def test_plain_no_binaries_or_services(self):

        y = meta.compose_package_yaml(self.config_data, ['armhf', 'amd64'])

        self.assertEqual(y['name'], 'my-package')
        self.assertEqual(y['version'], '1.0')
        self.assertEqual(y['vendor'], 'Sergio Schvezov <sergio.schvezov@canonical.com>')
        self.assertFalse('architecture' in y)
        self.assertTrue('amd64' in y['architectures'])
        self.assertTrue('armhf' in y['architectures'])
        self.assertEqual(len(y['architectures']), 2)
        self.assertFalse('binaries' in y)
        self.assertFalse('services' in y)

    def test_plain_no_binaries_or_services_or_arches(self):

        y = meta.compose_package_yaml(self.config_data, None)

        self.assertEqual(y['name'], 'my-package')
        self.assertEqual(y['version'], '1.0')
        self.assertEqual(y['vendor'], 'Sergio Schvezov <sergio.schvezov@canonical.com>')
        self.assertFalse('architectures' in y)
        self.assertFalse('architecture' in y)
        self.assertFalse('binaries' in y)
        self.assertFalse('services' in y)

    def test_with_binaries(self):
        self.config_data['binaries'] = [
            {
                'name': 'binary1',
                'exec': 'binary1.sh go',
            },
            {
                'name': 'binary2',
            },
        ]

        y = meta.compose_package_yaml(self.config_data, ['armhf', 'amd64'])

        self.assertEqual(y['name'], 'my-package')
        self.assertEqual(y['version'], '1.0')
        self.assertEqual(y['vendor'], 'Sergio Schvezov <sergio.schvezov@canonical.com>')
        self.assertTrue('amd64' in y['architectures'])
        self.assertTrue('armhf' in y['architectures'])
        self.assertEqual(len(y['architectures']), 2)
        self.assertFalse('services' in y)
        self.assertEquals(len(y['binaries']), 2)
        self.assertEqual(y['binaries'][0]['name'], 'binary1')
        self.assertEqual(y['binaries'][0]['exec'], 'binary.wrapped go')
        self.assertEqual(y['binaries'][1]['name'], 'binary2')
        self.assertEqual(y['binaries'][1]['exec'], 'binary.wrapped')

    def test_with_services(self):
        self.config_data['services'] = [
            {
                'name': 'service1',
                'start': 'binary1',
            },
            {
                'name': 'service2',
                'start': 'binary2 --start',
                'stop': 'binary2 --stop',
            },
            {
                'name': 'service3',
            },
        ]

        y = meta.compose_package_yaml(self.config_data, ['armhf', 'amd64'])

        self.assertEqual(y['name'], 'my-package')
        self.assertEqual(y['version'], '1.0')
        self.assertEqual(y['vendor'], 'Sergio Schvezov <sergio.schvezov@canonical.com>')
        self.assertTrue('amd64' in y['architectures'])
        self.assertTrue('armhf' in y['architectures'])
        self.assertEqual(len(y['architectures']), 2)
        self.assertFalse('binaries' in y)
        self.assertEqual(len(y['services']), 3)
        self.assertEqual(y['services'][0]['name'], 'service1')
        self.assertEqual(y['services'][0]['start'], 'binary.wrapped')
        self.assertFalse('stop' in y['services'][0])
        self.assertEqual(y['services'][1]['name'], 'service2')
        self.assertEqual(y['services'][1]['start'], 'binary.wrapped --start')
        self.assertEqual(y['services'][1]['stop'], 'binary.wrapped --stop')
        self.assertEqual(y['services'][2]['name'], 'service3')
        self.assertFalse('stop' in y['services'][2])
        self.assertFalse('stop' in y['services'][2])

    def test_compose_readme(self):
        self.config_data['summary'] = 'one line summary'
        self.config_data['description'] = 'the description\nwhich can be longer'

        readme_text = '''one line summary
the description
which can be longer
'''

        self.assertEqual(meta.compose_readme(self.config_data), readme_text)


class Create(tests.TestCase):

    def setUp(self):
        self.orig_os_makedirs = os.makedirs

        self.config_data = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'description': 'my description',
            'summary': 'my summary',
        }

    def tearDown(self):
        os.makedirs = self.orig_os_makedirs

    def test_create_meta(self):
        os.makedirs = Mock()
        mock_the_open = mock_open()

        with patch('snapcraft.meta.open', mock_the_open, create=True):
            meta.create(self.config_data, ['amd64'])

        meta_dir = os.path.join(os.path.abspath(os.curdir), 'snap', 'meta')

        mock_the_open.assert_has_calls([
            call(os.path.join(meta_dir, 'package.yaml'), 'w'),
            call().__enter__(),
            call().write('architectures'),
            call().write(':'),
            call().write('\n'),
            call().write('-'),
            call().write(' '),
            call().write('amd64'),
            call().write('\n'),
            call().write('name'),
            call().write(':'),
            call().write(' '),
            call().write('my-package'),
            call().write('\n'),
            call().write('vendor'),
            call().write(':'),
            call().write(' '),
            call().write('Sergio'),
            call().write(' '),
            call().write('Schvezov'),
            call().write(' '),
            call().write('<sergio.schvezov@canonical.com>'),
            call().write('\n'),
            call().write('version'),
            call().write(':'),
            call().write(" '"),
            call().write('1.0'),
            call().write("'"),
            call().write('\n'),
            call().flush(),
            call().flush(),
            call().__exit__(None, None, None),
            call(os.path.join(meta_dir, 'readme.md'), 'w'),
            call().__enter__(),
            call().write('my summary\nmy description\n'),
            call().__exit__(None, None, None),
        ]
        )


class WrapExeTestCase(tests.TestCase):

    def test_wrap_exe_must_write_wrapper(self):
        snapdir = common.get_snapdir()
        os.mkdir(snapdir)
        relative_exe_path = 'test_relexepath'
        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n'
                    'exec "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path, 'r') as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
