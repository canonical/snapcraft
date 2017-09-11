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

import json
import os
import subprocess
from unittest import mock

import fixtures


class FakePip(fixtures.Fixture):

    def __init__(self):
        super().__init__()
        self.download = {
            'disable_pip_version_check': False,
            'dest': None,
            'requirements': [],
            'constraints': [],
            'packages': []
        }
        self.install = {
            'disable_pip_version_check': False,
            'user': False,
            'no_compile': False,
            'no_index': False,
            'find_links': False,
            'ignore_installed': False,
            'installed': []
        }
        self.local_setup_file = False

    def setUp(self):
        super().setUp()

        original_call = subprocess.call
        original_check_output = subprocess.check_output

        def side_effect_call(cmd, *args, **kwargs):
            return side_effect(original_call, cmd, *args, **kwargs)

        def side_effect_check_output(cmd, *args, **kwargs):
            return side_effect(original_check_output, cmd, *args, **kwargs)

        def side_effect(original, cmd, *args, **kwargs):
            if self._is_pip_command(cmd):
                return self._fake_pip_command(cmd, *args, **kwargs)
            else:
                return original(cmd, *args, **kwargs)

        call_patcher = mock.patch(
            'subprocess.call', side_effect=side_effect_call)
        self.mock_call = call_patcher.start()
        self.addCleanup(call_patcher.stop)

        check_output_patcher = mock.patch(
            'subprocess.check_output', side_effect=side_effect_check_output)
        check_output_patcher.start()
        self.addCleanup(check_output_patcher.stop)

    def get_downloaded(self):
        return (self.download['constraints'] +
                self.download['requirements'] +
                self.download['packages'])

    def _is_pip_command(self, cmd):
        for index, arg in enumerate(cmd):
            if arg.endswith('/usr/bin/python3'):
                if cmd[index + 1] == '-m' and cmd[index + 2] == 'pip':
                    return True

        return False

    def _fake_pip_command(self, cmd, *args, **kwargs):
        if 'download' in cmd:
            return self._download(
                *cmd[cmd.index('download') + 1:], **kwargs)
        elif 'install' in cmd:
            return self._install(
                *cmd[cmd.index('install') + 1:], **kwargs)
        elif 'list' in cmd:
            return json.dumps([
                {'version': 'dummy', 'name': package}
                for package in self._list()]).encode('utf-8')

    def _download(self, *args, **kwargs):
        args = list(args)
        if '--disable-pip-version-check' in args:
            self.download['disable_pip_version_check'] = True
            args.remove('--disable-pip-version-check')
        if '--dest' in args:
            index = args.index('--dest')
            self.download['dest'] = args[index + 1]
            args.remove(args[index])
            args.remove(args[index])
        if '--constraint' in args:
            index = args.index('--constraint')
            with open(args[index + 1]) as constraint:
                self.download['constraints'] = constraint.read().splitlines()
            args.remove(args[index])
            args.remove(args[index])
        if '--requirement' in args:
            index = args.index('--requirement')
            with open(args[index + 1]) as requirement:
                self.download['requirements'] = requirement.read().splitlines()
            args.remove(args[index])
            args.remove(args[index])
        if args[0] == '.':
            if not os.path.exists(os.path.join(kwargs['cwd'], 'setup.py')):
                raise subprocess.CalledProcessError(returncode=1, cmd='dummy')
            args.remove('.')
            self.local_setup_file = True
        self.download['packages'] = args

    def _install(self, *args, **kwargs):
        args = list(args)
        for flag in [
                '--user', '--no-compile', '--disable-pip-version-check',
                '--no-index', '--ignore-installed']:
            if flag in args:
                self.install[flag.lstrip('-').replace('-', '_')] = True
                args.remove(flag)
        if '--find-links' in args:
            index = args.index('--find-links')
            # TODO test find-links
            args.remove(args[index])
            args.remove(args[index])
        self.install['installed'] = args

    def _list(self):
        # TODO
        return []
