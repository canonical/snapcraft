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

from snapcraft.internal.meta import create_snap_packaging, _SnapPackaging
from snapcraft.internal import common
from snapcraft.internal.errors import MissingGadgetError
from snapcraft import tests


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
        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

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

    def test_create_meta_with_confinement(self):
        confinement_types = [
            'strict',
            'devmode',
        ]

        for confinement_type in confinement_types:
            with self.subTest(key=confinement_type):
                self.config_data['confinement'] = confinement_type

                create_snap_packaging(
                    self.config_data, self.snap_dir, self.parts_dir)

                self.assertTrue(
                    os.path.exists(self.snap_yaml),
                    'snap.yaml was not created')

                with open(self.snap_yaml) as f:
                    y = yaml.load(f)
                self.assertTrue(
                    'confinement' in y,
                    'Expected "confinement" property to be in snap.yaml')
                self.assertEqual(y['confinement'], confinement_type)

    def test_create_meta_with_grade(self):
        grade_types = [
            'stable',
            'devel',
        ]

        for grade_type in grade_types:
            with self.subTest(key=grade_type):
                self.config_data['grade'] = grade_type

                create_snap_packaging(
                    self.config_data, self.snap_dir, self.parts_dir)

                self.assertTrue(
                    os.path.exists(self.snap_yaml),
                    'snap.yaml was not created')

                with open(self.snap_yaml) as f:
                    y = yaml.load(f)
                self.assertTrue(
                    'grade' in y,
                    'Expected "grade" property to be in snap.yaml')
                self.assertEqual(y['grade'], grade_type)

    def test_create_meta_with_epoch(self):
        self.config_data['epoch'] = '1*'

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertTrue(
            'epoch' in y,
            'Expected "epoch" property to be copied into snap.yaml')
        self.assertEqual(y['epoch'], '1*')

    def test_create_meta_with_assumes(self):
        self.config_data['assumes'] = ['feature1', 'feature2']

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)
        self.assertTrue(
            'assumes' in y,
            'Expected "assumes" property to be copied into snap.yaml')
        self.assertEqual(y['assumes'], ['feature1', 'feature2'])

    def test_create_gadget_meta_with_gadget_yaml(self):
        gadget_yaml = 'stub entry: stub value'
        with open(os.path.join('gadget.yaml'), 'w') as f:
            f.write(gadget_yaml)

        self.config_data['type'] = 'gadget'
        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        expected_gadget = os.path.join(self.meta_dir, 'gadget.yaml')
        self.assertTrue(os.path.exists(expected_gadget))

        with open(expected_gadget) as f:
            self.assertEqual(f.read(), gadget_yaml)

    def test_create_gadget_meta_with_missing_gadget_yaml_raises_error(self):
        self.config_data['type'] = 'gadget'

        with self.assertRaises(MissingGadgetError):
            create_snap_packaging(self.config_data,
                                  self.snap_dir,
                                  self.parts_dir)

    def test_create_meta_with_declared_icon(self):
        open(os.path.join(os.curdir, 'my-icon.png'), 'w').close()
        self.config_data['icon'] = 'my-icon.png'

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

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
        setup_icon_content = b'setup icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(setup_icon_content)

        declared_icon_content = b'declared icon'
        with open('my-icon.png', 'wb') as f:
            f.write(declared_icon_content)
        self.config_data['icon'] = 'my-icon.png'

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        with open(expected_icon, 'rb') as f:
            self.assertEqual(f.read(), declared_icon_content)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

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

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        # Running again should be good
        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

    def test_create_meta_with_icon_in_setup(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = b'this is the icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(icon_content)

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

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

    def test_create_meta_with_app(self):
        os.mkdir(self.snap_dir)
        open(os.path.join(self.snap_dir, 'app.sh'), 'w').close()
        self.config_data['apps'] = {
            'app1': {'command': 'app.sh'},
            'app2': {'command': 'app.sh', 'plugs': ['network']},
            'app3': {'command': 'app.sh', 'plugs': ['network-server']}
        }
        self.config_data['plugs'] = {
            'network-server': {'interface': 'network-bind'}}

        create_snap_packaging(self.config_data, self.snap_dir, self.parts_dir)

        for app in ['app1', 'app2', 'app3']:
            app_wrapper_path = os.path.join(
                self.snap_dir, 'command-{}.wrapper'.format(app))
            self.assertTrue(
                os.path.exists(app_wrapper_path),
                'the wrapper for {!r} was not setup correctly'.format(app))

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            y = yaml.load(f)

        expected = {
            'architectures': ['amd64'],
            'apps': {
                'app1': {
                    'command': 'command-app1.wrapper',
                },
                'app2': {
                    'command': 'command-app2.wrapper',
                    'plugs': ['network'],
                },
                'app3': {
                    'command': 'command-app3.wrapper',
                    'plugs': ['network-server'],
                },
            },
            'description': 'my description',
            'summary': 'my summary',
            'name': 'my-package',
            'version': '1.0',
            'plugs': {
                'network-server': {
                    'interface': 'network-bind',
                }
            }
        }

        self.assertEqual(y, expected)


# TODO this needs more tests.
class WrapExeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        # TODO move to use outer interface
        self.packager = _SnapPackaging({}, self.snap_dir, self.parts_dir)

    @patch('snapcraft.internal.common.assemble_env')
    def test_wrap_exe_must_write_wrapper(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(self.parts_dir)

        relative_exe_path = 'test_relexepath'
        open(os.path.join(self.snap_dir, relative_exe_path), 'w').close()

        # Check that the wrapper is created even if there is already a file
        # with the same name.
        open(os.path.join('prime', 'test_relexepath.wrapper'), 'w').close()

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')

        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    @patch('snapcraft.internal.common.assemble_env')
    def test_wrap_exe_writes_wrapper_with_basename(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(self.parts_dir)

        relative_exe_path = 'test_relexepath'
        open(os.path.join(self.snap_dir, relative_exe_path), 'w').close()

        relative_wrapper_path = self.packager._wrap_exe(
            relative_exe_path, basename='new-name')
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        self.assertEqual(relative_wrapper_path, 'new-name.wrapper')

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
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
        relative_exe_path = 'test_relexepath'
        shebang_path = os.path.join(
            self.parts_dir, 'testsnap', 'install', 'snap_exe')
        exe_contents = '#!{}\n'.format(shebang_path)
        with open(os.path.join(self.snap_dir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        expected = (
            '#!/bin/sh\n'
            '\n\n'
            'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
            'exec "$SNAP/snap_exe"'
            ' "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(self.snap_dir, relative_exe_path), 'r') as exe:
            # The shebang wasn't changed, since we don't know what the
            # path will be on the installed system.
            self.assertEqual(exe_contents, exe.read())

    def test_non_snap_shebangs_ignored(self):
        """Shebangs not pointing to the snap's install dir are ignored.

        If the shebang points to a system executable, there's no need to
        interfere.
        """
        relative_exe_path = 'test_relexepath'
        exe_contents = '#!/bin/bash\necho hello\n'
        with open(os.path.join(self.snap_dir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(self.snap_dir, relative_exe_path), 'r') as exe:
            self.assertEqual(exe_contents, exe.read())

    def test_non_shebang_binaries_ignored(self):
        """Native binaries are ignored.

        If the executable is a native binary, and thus not have a
        shebang, it's ignored.
        """
        relative_exe_path = 'test_relexepath'
        # Choose a content which can't be decoded with utf-8, to make
        # sure no decoding errors happen.
        exe_contents = b'\xf0\xf1'
        with open(os.path.join(self.snap_dir, relative_exe_path), 'wb') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(self.snap_dir, relative_exe_path), 'rb') as exe:
            self.assertEqual(exe_contents, exe.read())

    @patch('snapcraft.internal.common.run')
    def test_exe_is_in_path(self, run_mock):
        app_path = os.path.join(self.snap_dir, 'bin', 'app1')
        os.mkdir(os.path.dirname(app_path))
        open(app_path, 'w').close()

        relative_wrapper_path = self.packager._wrap_exe('app1')
        wrapper_path = os.path.join(self.snap_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "app1" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    def test_command_does_not_exist(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.snap_dir)]

        apps = {'app1': {'command': 'command-does-not-exist'}}

        with self.assertRaises(EnvironmentError) as raised:
            self.packager._wrap_apps(apps)
        self.assertEqual(
            "The specified command 'command-does-not-exist' defined in the "
            "app 'app1' does not exist or is not executable",
            str(raised.exception))

    def test_command_is_not_executable(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.snap_dir)]

        apps = {'app1': {'command': 'command-not-executable'}}

        cmd_path = os.path.join(self.snap_dir, 'bin', apps['app1']['command'])
        os.mkdir(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()

        with self.assertRaises(EnvironmentError) as raised:
            self.packager._wrap_apps(apps)
        self.assertEqual(
            "The specified command 'command-not-executable' defined in the "
            "app 'app1' does not exist or is not executable",
            str(raised.exception))

    def test_command_found(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.snap_dir)]

        apps = {'app1': {'command': 'command-executable'}}

        cmd_path = os.path.join(self.snap_dir, 'bin', apps['app1']['command'])
        os.mkdir(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()
        os.chmod(cmd_path, 0o755)

        wrapped_apps = self.packager._wrap_apps(apps)

        self.assertEqual(wrapped_apps,
                         {'app1': {'command': 'command-app1.wrapper'}})
