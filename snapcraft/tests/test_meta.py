# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import os
from unittest.mock import patch

import fixtures
import yaml

from snapcraft import (
    common,
    meta,
    tests
)


class CreateTest(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.config_data = {
            'architectures': ['amd64'],
            'name': 'my-package',
            'version': '1.0',
            'description': 'my description',
            'summary': 'my summary',
        }

        self.snap_dir = os.path.join(os.path.abspath(os.curdir), 'snap')
        self.meta_dir = os.path.join(self.snap_dir, 'meta')
        self.hooks_dir = os.path.join(self.meta_dir, 'hooks')
        self.snap_yaml = os.path.join(self.meta_dir, 'snap.yaml')

    def test_create_meta(self):
        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_meta_with_declared_license(self):
        open(os.path.join(os.curdir, 'LICENSE'), 'w').close()
        self.config_data['license'] = 'LICENSE'

        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(os.path.join(self.meta_dir, 'license.txt')),
            'license.txt was not setup correctly')

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('license' in y,
                         'license found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_license_and_setup(self):
        open(os.path.join(os.curdir, 'LICENSE'), 'w').close()
        self.config_data['license'] = 'LICENSE'

        os.mkdir('setup')
        license_text = 'this is the license'
        with open(os.path.join('setup', 'license.txt'), 'w') as f:
            f.write(license_text)

        meta.create(self.config_data)

        expected_license = os.path.join(self.meta_dir, 'license.txt')
        self.assertTrue(os.path.exists(expected_license),
                        'license.txt was not setup correctly')
        with open(expected_license) as f:
            self.assertEqual(f.read(), license_text)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')
        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('license' in y,
                         'license found in snap.yaml {}'.format(y))

    def test_create_meta_with_license_in_setup(self):
        os.mkdir('setup')
        license_text = 'this is the license'
        with open(os.path.join('setup', 'license.txt'), 'w') as f:
            f.write(license_text)

        meta.create(self.config_data)

        expected_license = os.path.join(self.meta_dir, 'license.txt')
        self.assertTrue(os.path.exists(expected_license),
                        'license.txt was not setup correctly')
        with open(expected_license) as f:
            self.assertEqual(f.read(), license_text)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')
        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('license' in y,
                         'license found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon(self):
        open(os.path.join(os.curdir, 'my-icon.png'), 'w').close()
        self.config_data['icon'] = 'my-icon.png'

        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(os.path.join(self.meta_dir, 'gui', 'icon.png')),
            'icon.png was not setup correctly')

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon_and_setup(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = b'this is the icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(icon_content)

        open(os.path.join(os.curdir, 'my-icon.png'), 'w').close()
        self.config_data['icon'] = 'my-icon.png'

        meta.create(self.config_data)

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        with open(expected_icon, 'rb') as f:
            self.assertEqual(f.read(), icon_content)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        self.assertTrue(
            "DEPRECATED: 'icon' defined in snapcraft.yaml"
            in fake_logger.output, 'Missing deprecation message for icon')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon_and_setup_ran_twice_ok(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = b'this is the icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(icon_content)

        open(os.path.join(os.curdir, 'my-icon.png'), 'w').close()
        self.config_data['icon'] = 'my-icon.png'

        meta.create(self.config_data)

        # Running again should be good
        meta.create(self.config_data)

    def test_create_meta_with_icon_in_setup(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = b'this is the icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(icon_content)

        meta.create(self.config_data)

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        with open(expected_icon, 'rb') as f:
            self.assertEqual(f.read(), icon_content)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_config(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'config.sh'), 'w').close()
        self.config_data['config'] = 'config.sh'

        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(os.path.join(self.hooks_dir, 'config')),
            'the config was not setup correctly')

    def test_create_meta_with_config_with_args(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'config.sh'), 'w').close()
        self.config_data['config'] = 'config.sh something'

        meta.create(self.config_data)

        config_hook = os.path.join(self.hooks_dir, 'config')
        self.assertTrue(
            os.path.exists(config_hook), 'the config was not setup correctly')

        with open(config_hook) as f:
            config_wrapper = f.readlines()

        expected_wrapper = [
            '#!/bin/sh\n', '\n', '\n',
            'exec "$SNAP/config.sh" something $*\n']
        self.assertEqual(config_wrapper, expected_wrapper)

    def test_create_meta_with_app(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app1.sh'), 'w').close()
        self.config_data['apps'] = {
            'app1': {'command': 'app1.sh'},
        }

        meta.create(self.config_data)

        app1_wrapper_path = os.path.join(self.snap_dir, 'command-app1.wrapper')
        self.assertTrue(
            os.path.exists(app1_wrapper_path),
            'the wrapper for app1 was not setup correctly')

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'apps': {
                        'app1': {
                            'command': 'command-app1.wrapper',
                        },
                    },
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_meta_with_app_with_security_policy(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app1.sh'), 'w').close()
        open(os.path.join(os.curdir, 'stub-sec'), 'w').close()
        self.config_data['apps'] = {
            'app1': {
                'command': 'app1.sh',
                'uses': ['migration'],
            },
        }
        self.config_data['uses'] = {
            'migration': {
                'type': 'migration-skill',
                'security-policy': {
                    'apparmor': 'stub-sec',
                    'seccomp': 'stub-sec',
                },
            },
        }

        meta.create(self.config_data)

        app1_wrapper_path = os.path.join(self.snap_dir, 'command-app1.wrapper')
        self.assertTrue(
            os.path.exists(app1_wrapper_path),
            'the wrapper for app1 was not setup correctly')

        sec_path = os.path.join(self.meta_dir, 'stub-sec')
        self.assertTrue(
            os.path.exists(sec_path),
            'the security-policies for app1 were not setup correctly')

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'apps': {
                        'app1': {
                            'command': 'command-app1.wrapper',
                            'uses': ['migration'],
                        },
                    },
                    'uses': {
                        'migration': {
                            'type': 'migration-skill',
                            'security-policy': {
                                'apparmor': 'meta/stub-sec',
                                'seccomp': 'meta/stub-sec',
                            },
                        }
                    },
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_meta_with_implicit_migration_skill(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app1.sh'), 'w').close()
        open(os.path.join(os.curdir, 'stub-sec'), 'w').close()
        self.config_data['apps'] = {
            'app1': {
                'command': 'app1.sh',
                'uses': ['migration-skill'],
            },
        }
        self.config_data['uses'] = {
            'migration-skill': {
                'security-policy': {
                    'apparmor': 'stub-sec',
                    'seccomp': 'stub-sec',
                },
            },
        }

        meta.create(self.config_data)

        app1_wrapper_path = os.path.join(self.snap_dir, 'command-app1.wrapper')
        self.assertTrue(
            os.path.exists(app1_wrapper_path),
            'the wrapper for app1 was not setup correctly')

        sec_path = os.path.join(self.meta_dir, 'stub-sec')
        self.assertTrue(
            os.path.exists(sec_path),
            'the security-policies for app1 were not setup correctly')

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'apps': {
                        'app1': {
                            'command': 'command-app1.wrapper',
                            'uses': ['migration-skill'],
                        },
                    },
                    'uses': {
                        'migration-skill': {
                            'security-policy': {
                                'apparmor': 'meta/stub-sec',
                                'seccomp': 'meta/stub-sec',
                            },
                        }
                    },
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_no_change_if_not_migration_skill(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app1.sh'), 'w').close()
        open(os.path.join(os.curdir, 'stub-sec'), 'w').close()
        self.config_data['apps'] = {
            'app1': {
                'command': 'app1.sh',
                'uses': ['migration'],
            },
        }
        self.config_data['uses'] = {
            'migration': {
                'type': 'not-a-migration-skillz',
                'security-policy': {
                    'apparmor': 'stub-sec',
                    'seccomp': 'stub-sec',
                },
            },
        }

        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'apps': {
                        'app1': {
                            'command': 'command-app1.wrapper',
                            'uses': ['migration'],
                        },
                    },
                    'uses': {
                        'migration': {
                            'type': 'not-a-migration-skillz',
                            'security-policy': {
                                'apparmor': 'stub-sec',
                                'seccomp': 'stub-sec',
                            },
                        }
                    },
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)

    def test_create_with_migration_skill_with_caps(self):
        os.makedirs(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app1.sh'), 'w').close()
        open(os.path.join(os.curdir, 'stub-sec'), 'w').close()
        self.config_data['apps'] = {
            'app1': {
                'command': 'app1.sh',
                'uses': ['migration'],
            },
        }
        self.config_data['uses'] = {
            'migration': {
                'type': 'migration-skill',
                'caps': ['network-listener'],
            },
        }

        meta.create(self.config_data)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {'architectures': ['amd64'],
                    'apps': {
                        'app1': {
                            'command': 'command-app1.wrapper',
                            'uses': ['migration'],
                        },
                    },
                    'uses': {
                        'migration': {
                            'type': 'migration-skill',
                            'caps': ['network-listener'],
                        }
                    },
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected)


# TODO this needs more tests.
class WrapExeTestCase(tests.TestCase):

    @patch('snapcraft.common.assemble_env')
    def test_wrap_exe_must_write_wrapper(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(common.get_partsdir())

        snapdir = common.get_snapdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        open(os.path.join(snapdir, relative_exe_path), 'w').close()

        # Check that the wrapper is created even if there is already a file
        # with the same name.
        open(os.path.join('snap', 'test_relexepath.wrapper'), 'w').close()

        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n'
                    '\n\n'
                    'exec "$SNAP/test_relexepath" $*\n')

        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    @patch('snapcraft.common.assemble_env')
    def test_wrap_exe_writes_wrapper_with_basename(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(common.get_partsdir())

        snapdir = common.get_snapdir()
        os.mkdir(snapdir)

        relative_exe_path = 'test_relexepath'
        open(os.path.join(snapdir, relative_exe_path), 'w').close()

        relative_wrapper_path = meta._wrap_exe(
            relative_exe_path, basename='new-name')
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        self.assertEqual(relative_wrapper_path, 'new-name.wrapper')

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n'
                    '\n\n'
                    'exec "$SNAP/test_relexepath" $*\n')
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
        it in the wrapper script allows us to use the $SNAP environment
        variable.
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
            'exec "$SNAP/snap_exe"'
            ' "$SNAP/test_relexepath" $*\n')
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
                    'exec "$SNAP/test_relexepath" $*\n')
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
                    'exec "$SNAP/test_relexepath" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(snapdir, relative_exe_path), 'rb') as exe:
            self.assertEqual(exe_contents, exe.read())

    @patch('snapcraft.common.run')
    def test_exe_is_in_path(self, run_mock):
        snapdir = common.get_snapdir()
        app_path = os.path.join(snapdir, 'bin', 'app1')
        os.makedirs(os.path.dirname(app_path))
        open(app_path, 'w').close()

        relative_wrapper_path = meta._wrap_exe('app1')
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'exec "app1" $*\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    def test_command_does_not_exist(self):
        snap_dir = common.get_snapdir()
        common.env = ['PATH={}/bin:$PATH'.format(snap_dir)]

        os.mkdir(snap_dir)
        apps = {'app1': {'command': 'command-does-not-exist'}}

        with self.assertRaises(EnvironmentError) as raised:
            meta._wrap_apps(apps)
        self.assertEqual(
            "The specified command 'command-does-not-exist' defined in 'app1' "
            "does not exist or is not executable",
            str(raised.exception))

    def test_command_is_not_executable(self):
        snap_dir = common.get_snapdir()
        common.env = ['PATH={}/bin:$PATH'.format(snap_dir)]

        apps = {'app1': {'command': 'command-not-executable'}}

        cmd_path = os.path.join(snap_dir, 'bin', apps['app1']['command'])
        os.makedirs(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()

        with self.assertRaises(EnvironmentError) as raised:
            meta._wrap_apps(apps)
        self.assertEqual(
            "The specified command 'command-not-executable' defined in 'app1' "
            "does not exist or is not executable",
            str(raised.exception))

    def test_command_found(self):
        snap_dir = common.get_snapdir()
        common.env = ['PATH={}/bin:$PATH'.format(snap_dir)]

        apps = {'app1': {'command': 'command-executable'}}

        cmd_path = os.path.join(snap_dir, 'bin', apps['app1']['command'])
        os.makedirs(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()
        os.chmod(cmd_path, 0o755)

        wrapped_apps = meta._wrap_apps(apps)

        self.assertEqual(wrapped_apps,
                         {'app1': {'command': 'command-app1.wrapper'}})
