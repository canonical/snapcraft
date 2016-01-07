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

import os

import yaml

from snapcraft import (
    meta_legacy,
    tests
)


class ComposeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.snap_yaml = {
            'name': 'my-package',
            'version': '1.0',
            'summary': 'summary',
            'description': 'description',
            'architectures': ['armhf', 'amd64']
        }

    def test_plain_no_apps(self):
        y = meta_legacy._compose_package_yaml('meta', self.snap_yaml)

        expected = {
            'name': 'my-package',
            'version': '1.0',
            'architectures': ['armhf', 'amd64'],
        }

        self.assertEqual(y, expected)

    def test_plain_no_apps_or_arches(self):
        self.snap_yaml.pop('architectures')

        y = meta_legacy._compose_package_yaml('meta', self.snap_yaml)

        expected = {
            'name': 'my-package',
            'version': '1.0',
        }

        self.assertEqual(y, expected)

    def test_license_information(self):
        self.snap_yaml['license-agreement'] = 'explicit'
        self.snap_yaml['license-version'] = '1.0'

        y = meta_legacy._compose_package_yaml('meta', self.snap_yaml)

        expected = {
            'name': 'my-package',
            'version': '1.0',
            'architectures': ['armhf', 'amd64'],
            'license-version': '1.0',
            'explicit-license-agreement': 'yes',
        }

        self.assertEqual(y, expected)

    def test_with_cli_apps(self):
        self.snap_yaml['apps'] = {
            'binary1': {'command': 'binary1.sh go'},
            'binary2': {'command': 'binary2.sh'},
        }

        y = meta_legacy._compose_package_yaml('meta', self.snap_yaml)

        self.assertEqual(len(y['binaries']), 2)
        for b in y['binaries']:
            if b['name'] is 'binary1':
                self.assertEqual(b['exec'], 'binary1.sh go')
            else:
                self.assertEqual(b['exec'], 'binary2.sh')

    def test_with_daemon_apps(self):
        self.snap_yaml['apps'] = {
            'service1': {'command': 'binary1', 'daemon': 'simple'},
            'service2': {
                'command': 'binary2 --start',
                'stop-command': 'binary2 --stop',
                'daemon': 'forking',
            },
        }

        y = meta_legacy._compose_package_yaml('meta', self.snap_yaml)

        self.assertEqual(len(y['services']), 2)
        for b in y['services']:
            if b['name'] is 'service1':
                self.assertEqual(b['start'], 'binary1')
            else:
                self.assertEqual(b['start'], 'binary2 --start')
                self.assertEqual(b['stop'], 'binary2 --stop')
                self.assertEqual(b['forking'], 'yes')

    def test_compose_readme(self):
        self.snap_yaml['summary'] = 'one line summary'
        self.snap_yaml['description'] = 'the description\nwhich can be longer'

        readme_text = '''one line summary
the description
which can be longer
'''

        self.assertEqual(meta_legacy._compose_readme(self.snap_yaml),
                         readme_text)


class Create(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.snap_yaml = {
            'name': 'my-package',
            'version': '1.0',
            'description': 'my description',
            'summary': 'my summary',
            'architectures': ['amd64'],
            'apps': {
                'bash': {
                    'command': 'bin/bash',
                    'security-policy': {
                        'apparmor': 'file.apparmor',
                        'seccomp': 'file.seccomp',
                    },
                }
            }
        }

        self.snap_dir = os.path.join(os.path.abspath(os.curdir), 'snap')
        self.meta_dir = os.path.join(self.snap_dir, 'meta')
        self.hooks_dir = os.path.join(self.meta_dir, 'hooks')
        self.package_yaml = os.path.join(self.meta_dir, 'package.yaml')
        os.makedirs(self.meta_dir)

    def test_create_meta(self):
        meta_legacy.create(self.meta_dir, self.snap_yaml)

        self.assertTrue(
            os.path.exists(self.package_yaml), 'package.yaml was not created')

        with open(self.package_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'binaries': [{
                        'exec': 'bin/bash',
                        'name': 'bash',
                        'security-policy': {
                            'apparmor': 'file.apparmor',
                            'seccomp': 'file.seccomp'}}],
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_with_png_icon(self):
        open(os.path.join(self.meta_dir, 'icon.png'), 'w').close()

        meta_legacy.create(self.meta_dir, self.snap_yaml)

        self.assertTrue(
            os.path.exists(self.package_yaml), 'package.yaml was not created')

        with open(self.package_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'binaries': [{
                        'exec': 'bin/bash',
                        'name': 'bash',
                        'security-policy': {
                            'apparmor': 'file.apparmor',
                            'seccomp': 'file.seccomp'}}],
                    'icon': 'meta/icon.png',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_with_svg_icon(self):
        open(os.path.join(self.meta_dir, 'icon.svg'), 'w').close()

        meta_legacy.create(self.meta_dir, self.snap_yaml)

        self.assertTrue(
            os.path.exists(self.package_yaml), 'package.yaml was not created')

        with open(self.package_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'binaries': [{
                        'exec': 'bin/bash',
                        'name': 'bash',
                        'security-policy': {
                            'apparmor': 'file.apparmor',
                            'seccomp': 'file.seccomp'}}],
                    'icon': 'meta/icon.svg',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)
