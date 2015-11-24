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
    call,
    patch,
)

from snapcraft import (
    common,
    meta,
    tests
)


class ComposeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        patcher = patch('snapcraft.meta._wrap_exe')
        mock_wrap_exe = patcher.start()
        mock_wrap_exe.return_value = 'binary.wrapped'
        self.addCleanup(patcher.stop)

        self.config_data = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'icon': 'my-icon.png',
        }

    def test_plain_no_binaries_or_services(self):
        y = meta._compose_package_yaml('meta', self.config_data,
                                       ['armhf', 'amd64'])

        expected = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'icon': 'my-icon.png',
            'architectures': ['armhf', 'amd64'],
        }

        self.assertEqual(y, expected)

    def test_plain_no_binaries_or_services_or_arches(self):
        y = meta._compose_package_yaml('meta', self.config_data, None)

        expected = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'icon': 'my-icon.png',
        }

        self.assertEqual(y, expected)

    def test_with_binaries(self):
        self.config_data['binaries'] = {
            'binary1': {'exec': 'binary1.sh go'},
            'binary2': {'exec': 'binary2.sh'},
        }

        y = meta._compose_package_yaml('meta', self.config_data,
                                       ['armhf', 'amd64'])

        self.assertEqual(len(y['binaries']), 2)
        for b in y['binaries']:
            if b['name'] is 'binary1':
                self.assertEqual(b['exec'], 'binary.wrapped go')
            else:
                self.assertEqual(b['exec'], 'binary.wrapped')

    def test_with_services(self):
        self.config_data['services'] = {
            'service1': {'start': 'binary1'},
            'service2': {
                'start': 'binary2 --start',
                'stop': 'binary2 --stop',
            },
        }

        y = meta._compose_package_yaml('meta', self.config_data,
                                       ['armhf', 'amd64'])

        self.assertEqual(len(y['services']), 2)
        for b in y['services']:
            if b['name'] is 'service1':
                self.assertEqual(b['start'], 'binary.wrapped')
            else:
                self.assertEqual(b['start'], 'binary.wrapped --start')
                self.assertEqual(b['stop'], 'binary.wrapped --stop')

    def test_plain_no_binaries_or_services_with_optionals(self):
        self.config_data['frameworks'] = ['mir', ]

        y = meta._compose_package_yaml('meta', self.config_data,
                                       ['armhf', 'amd64'])

        expected = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'icon': 'my-icon.png',
            'architectures': ['armhf', 'amd64'],
            'frameworks': ['mir', ],
        }

        self.assertEqual(y, expected)

    def test_compose_readme(self):
        self.config_data['summary'] = 'one line summary'
        self.config_data['description'] = \
            'the description\nwhich can be longer'

        readme_text = '''one line summary
the description
which can be longer
'''

        self.assertEqual(meta._compose_readme(self.config_data), readme_text)


class Create(tests.TestCase):

    def setUp(self):
        super().setUp()
        patcher_makedirs = patch('os.makedirs')
        self.mock_makedirs = patcher_makedirs.start()
        self.addCleanup(patcher_makedirs.stop)

        patcher_copyfile = patch('shutil.copyfile')
        self.mock_copyfile = patcher_copyfile.start()
        self.addCleanup(patcher_copyfile.stop)

        patcher_move = patch('shutil.move')
        self.mock_move = patcher_move.start()
        self.addCleanup(patcher_move.stop)

        patcher_exists = patch('os.path.exists')
        self.mock_exists = patcher_exists.start()
        self.mock_exists.return_value = True
        self.addCleanup(patcher_exists.stop)

        self.config_data = {
            'name': 'my-package',
            'version': '1.0',
            'vendor': 'Sergio Schvezov <sergio.schvezov@canonical.com>',
            'description': 'my description',
            'summary': 'my summary',
            'icon': 'my-icon.png',
            'config': 'bin/config',
            'binaries': {
                'bash': {
                    'exec': 'bin/bash',
                    'security-policy': {
                        'apparmor': 'file.apparmor',
                        'seccomp': 'file.seccomp',
                    },
                }
            }
        }

        self.meta_dir = os.path.join(os.path.abspath(os.curdir),
                                     'snap', 'meta')
        self.hooks_dir = os.path.join(self.meta_dir, 'hooks')

        self.expected_open_calls = [
            call(os.path.join(self.meta_dir, 'package.yaml'), 'w'),
            call().__enter__(),
            call().__enter__().write('architectures'),
            call().__enter__().write(':'),
            call().__enter__().write('\n'),
            call().__enter__().write('-'),
            call().__enter__().write(' '),
            call().__enter__().write('amd64'),
            call().__enter__().write('\n'),
            call().__enter__().write('binaries'),
            call().__enter__().write(':'),
            call().__enter__().write('\n'),
            call().__enter__().write('-'),
            call().__enter__().write(' '),
            call().__enter__().write('exec'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('bin/bash.wrapper'),
            call().__enter__().write('\n'),
            call().__enter__().write('  '),
            call().__enter__().write('name'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('bash'),
            call().__enter__().write('\n'),
            call().__enter__().write('  '),
            call().__enter__().write('security-policy'),
            call().__enter__().write(':'),
            call().__enter__().write('\n'),
            call().__enter__().write('    '),
            call().__enter__().write('apparmor'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('meta/file.apparmor'),
            call().__enter__().write('\n'),
            call().__enter__().write('    '),
            call().__enter__().write('seccomp'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('meta/file.seccomp'),
            call().__enter__().write('\n'),
            call().__enter__().write('icon'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('meta/my-icon.png'),
            call().__enter__().write('\n'),
            call().__enter__().write('name'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('my-package'),
            call().__enter__().write('\n'),
            call().__enter__().write('vendor'),
            call().__enter__().write(':'),
            call().__enter__().write(' '),
            call().__enter__().write('Sergio'),
            call().__enter__().write(' '),
            call().__enter__().write('Schvezov'),
            call().__enter__().write(' '),
            call().__enter__().write('<sergio.schvezov@canonical.com>'),
            call().__enter__().write('\n'),
            call().__enter__().write('version'),
            call().__enter__().write(':'),
            call().__enter__().write(" '"),
            call().__enter__().write('1.0'),
            call().__enter__().write("'"),
            call().__enter__().write('\n'),
            call().__enter__().flush(),
            call().__enter__().flush(),
            call().__exit__(None, None, None),
            call(os.path.join(self.meta_dir, 'readme.md'), 'w'),
            call().__enter__(),
            call().__enter__().write('my summary\nmy description\n'),
            call().__exit__(None, None, None),
        ]

    @patch('snapcraft.meta._write_wrap_exe')
    @patch('snapcraft.meta.open', create=True)
    def test_create_meta(self, mock_the_open, mock_wrap_exe):
        meta.create(self.config_data, ['amd64'])

        self.mock_makedirs.assert_has_calls([
            call(self.meta_dir, exist_ok=True),
            call(self.hooks_dir),
        ])

        mock_the_open.assert_has_calls(self.expected_open_calls)
        mock_wrap_exe.assert_has_calls([
            call(
                '$SNAP_APP_PATH/bin/bash',
                os.path.join(os.path.abspath(os.curdir),
                             'snap/bin/bash.wrapper'),
                shebang=None,
            ),
            call(
                'bin/config',
                os.path.join(self.hooks_dir, 'config'),
                args=[],
                cwd='$SNAP_APP_PATH',
            ),
        ])
        self.mock_copyfile.assert_has_calls([
            call('my-icon.png', os.path.join(self.meta_dir,
                 'my-icon.png')),
            call('file.apparmor', os.path.join(self.meta_dir,
                 'file.apparmor')),
            call('file.seccomp', os.path.join(self.meta_dir,
                 'file.seccomp')),
        ])

    @patch('snapcraft.meta._write_wrap_exe')
    @patch('snapcraft.meta.open', create=True)
    def test_create_meta_with_vararg_config(self, mock_the_open,
                                            mock_wrap_exe):
        self.config_data['config'] = 'python3 my.py --config'

        meta.create(self.config_data, ['amd64'])

        self.mock_makedirs.assert_has_calls([
            call(self.meta_dir, exist_ok=True),
            call(self.hooks_dir),
        ])

        mock_the_open.assert_has_calls(self.expected_open_calls)
        mock_wrap_exe.assert_has_calls([
            call(
                '$SNAP_APP_PATH/bin/bash',
                os.path.join(os.path.abspath(os.curdir),
                             'snap/bin/bash.wrapper'),
                shebang=None,
            ),
            call(
                'python3',
                os.path.join(self.hooks_dir, 'config'),
                args=['my.py', '--config'],
                cwd='$SNAP_APP_PATH',
            ),
        ])

    @patch('snapcraft.meta._write_wrap_exe')
    @patch('snapcraft.meta.open', create=True)
    def test_create_meta_without_config(self, mock_the_open, mock_wrap_exe):
        del self.config_data['config']

        meta.create(self.config_data, ['amd64'])

        self.mock_makedirs.assert_called_once_with(self.meta_dir,
                                                   exist_ok=True)
        mock_the_open.assert_has_calls(self.expected_open_calls)


# TODO this needs more tests.
class WrapExeTestCase(tests.TestCase):

    def test_wrap_exe_must_write_wrapper(self):
        snapdir = common.get_snapdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        with open(os.path.join(snapdir, relative_exe_path), 'w'):
            pass

        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'exec "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    def test_snap_shebangs_extracted(self):
        """Shebangs pointing to the snap's install dir get extracted.

        If the exe has a shebang that points to the snap's install dir,
        the wrapper script will execute it directly rather than relying
        on the shebang.

        The shebang needs to be an absolute path, and we don't know
        in which directory the snap will be installed. Executing
        it in the wrapper script allows us to use the $SNAP_APP_PATH
        environment variable.
        """
        snapdir = common.get_snapdir()
        partsdir = common.get_partsdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        shebang_path = os.path.join(
            partsdir, 'testsnap', 'install', 'snap_exe')
        exe_contents = '#!{}\n'.format(shebang_path)
        with open(os.path.join(snapdir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = (
            '#!/bin/sh\n'
            '\n\n'
            'exec "$SNAP_APP_PATH/snap_exe"'
            ' "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(snapdir, relative_exe_path), 'r') as exe:
            # The shebang wasn't changed, since we don't know what the
            # path will be on the installed system.
            self.assertEqual(exe_contents, exe.read())

    def test_non_snap_shebangs_ignored(self):
        """Shebangs not pointing to the snap's install dir are ignored.

        If the shebang points to a system executable, there's no need to
        interfere.
        """
        snapdir = common.get_snapdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        exe_contents = '#!/bin/bash\necho hello\n'
        with open(os.path.join(snapdir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'exec "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(snapdir, relative_exe_path), 'r') as exe:
            self.assertEqual(exe_contents, exe.read())

    def test_non_shebang_binaries_ignored(self):
        """Native binaries are ignored.

        If the executable is a native binary, and thus not have a
        shebang, it's ignored.
        """
        snapdir = common.get_snapdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        # Choose a content which can't be decoded with utf-8, to make
        # sure no decoding errors happen.
        exe_contents = b'\xf0\xf1'
        with open(os.path.join(snapdir, relative_exe_path), 'wb') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'exec "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(snapdir, relative_exe_path), 'rb') as exe:
            self.assertEqual(exe_contents, exe.read())
