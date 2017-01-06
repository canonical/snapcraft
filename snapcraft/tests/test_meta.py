# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import configparser
import logging
import os
from unittest.mock import patch
import testtools
from testtools.matchers import (
    Contains,
    Equals,
    FileContains,
    FileExists,
    HasLength,
    Not
)

import fixtures
import yaml

from snapcraft.internal.meta import (
    CommandError,
    create_snap_packaging,
    _SnapPackaging
)
from snapcraft.internal import common
from snapcraft.internal.errors import MissingGadgetError
from snapcraft import tests


class CreateBaseTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.config_data = {
            'architectures': ['amd64'],
            'name': 'my-package',
            'version': '1.0',
            'description': 'my description',
            'summary': 'my summary',
            'confinement': 'devmode',
        }

        self.snap_dir = os.path.join(os.path.abspath(os.curdir), 'snap')
        self.prime_dir = os.path.join(os.path.abspath(os.curdir), 'prime')
        self.meta_dir = os.path.join(self.prime_dir, 'meta')
        self.hooks_dir = os.path.join(self.meta_dir, 'hooks')
        self.snap_yaml = os.path.join(self.meta_dir, 'snap.yaml')

    def generate_meta_yaml(self):
        create_snap_packaging(self.config_data, self.prime_dir, self.parts_dir)

        self.assertTrue(
            os.path.exists(self.snap_yaml), 'snap.yaml was not created')

        with open(self.snap_yaml) as f:
            return yaml.load(f)


class CreateTestCase(CreateBaseTestCase):

    def test_create_meta(self):
        y = self.generate_meta_yaml()

        expected = {'architectures': ['amd64'],
                    'confinement': 'devmode',
                    'description': 'my description',
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertEqual(y, expected, expected)

    def test_create_meta_with_epoch(self):
        self.config_data['epoch'] = '1*'

        y = self.generate_meta_yaml()
        self.assertTrue(
            'epoch' in y,
            'Expected "epoch" property to be copied into snap.yaml')
        self.assertEqual(y['epoch'], '1*')

    def test_create_meta_with_assumes(self):
        self.config_data['assumes'] = ['feature1', 'feature2']

        y = self.generate_meta_yaml()
        self.assertTrue(
            'assumes' in y,
            'Expected "assumes" property to be copied into snap.yaml')
        self.assertEqual(y['assumes'], ['feature1', 'feature2'])

    def test_create_gadget_meta_with_gadget_yaml(self):
        gadget_yaml = 'stub entry: stub value'
        with open(os.path.join('gadget.yaml'), 'w') as f:
            f.write(gadget_yaml)

        self.config_data['type'] = 'gadget'
        create_snap_packaging(self.config_data, self.prime_dir, self.parts_dir)

        expected_gadget = os.path.join(self.meta_dir, 'gadget.yaml')
        self.assertTrue(os.path.exists(expected_gadget))

        with open(expected_gadget) as f:
            self.assertEqual(f.read(), gadget_yaml)

    def test_create_gadget_meta_with_missing_gadget_yaml_raises_error(self):
        self.config_data['type'] = 'gadget'

        self.assertRaises(
            MissingGadgetError,
            create_snap_packaging,
            self.config_data,
            self.prime_dir,
            self.parts_dir)

    def test_create_meta_with_declared_icon(self):
        open(os.path.join(os.curdir, 'my-icon.png'), 'w').close()
        self.config_data['icon'] = 'my-icon.png'

        y = self.generate_meta_yaml()

        self.assertTrue(
            os.path.exists(os.path.join(self.meta_dir, 'gui', 'icon.png')),
            'icon.png was not setup correctly')

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

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        with open(expected_icon, 'rb') as f:
            self.assertEqual(f.read(), declared_icon_content)

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

        create_snap_packaging(self.config_data, self.prime_dir, self.parts_dir)

        # Running again should be good
        create_snap_packaging(self.config_data, self.prime_dir, self.parts_dir)

    def test_create_meta_with_icon_in_setup(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = b'this is the icon'
        with open(os.path.join(gui_path, 'icon.png'), 'wb') as f:
            f.write(icon_content)

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        with open(expected_icon, 'rb') as f:
            self.assertEqual(f.read(), icon_content)

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_app(self):
        os.mkdir(self.prime_dir)
        open(os.path.join(self.prime_dir, 'app.sh'), 'w').close()
        self.config_data['apps'] = {
            'app1': {'command': 'app.sh'},
            'app2': {'command': 'app.sh', 'plugs': ['network']},
            'app3': {'command': 'app.sh', 'plugs': ['network-server']}
        }
        self.config_data['plugs'] = {
            'network-server': {'interface': 'network-bind'}}

        y = self.generate_meta_yaml()

        for app in ['app1', 'app2', 'app3']:
            app_wrapper_path = os.path.join(
                self.prime_dir, 'command-{}.wrapper'.format(app))
            self.assertTrue(
                os.path.exists(app_wrapper_path),
                'the wrapper for {!r} was not setup correctly'.format(app))

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
            'confinement': 'devmode',
            'plugs': {
                'network-server': {
                    'interface': 'network-bind',
                }
            }
        }

        self.assertEqual(y, expected)

    def test_create_meta_with_app_desktop_key(self):
        os.mkdir(self.prime_dir)
        open(os.path.join(self.prime_dir, 'app.sh'), 'w').close()
        with open(os.path.join(self.prime_dir, 'app1.desktop'), 'w') as f:
            f.write('[Desktop Entry]\nExec=app1.exe\nIcon=app1.png')
        icon_dir = os.path.join(self.prime_dir, 'usr', 'share')
        os.makedirs(icon_dir)
        open(os.path.join(icon_dir, 'app2.png'), 'w').close()
        with open(os.path.join(self.prime_dir, 'app2.desktop'), 'w') as f:
            f.write('[Desktop Entry]\nExec=app2.exe\nIcon=/usr/share/app2.png')
        self.config_data['apps'] = {
            'app1': {'command': 'app.sh', 'desktop': 'app1.desktop'},
            'app2': {'command': 'app.sh', 'desktop': 'app2.desktop'}
        }

        self.generate_meta_yaml()

        desktop_file = os.path.join(self.meta_dir, 'gui', 'app1.desktop')
        self.assertTrue(os.path.exists(desktop_file),
                        'app1.desktop was not setup correctly')
        contents = configparser.ConfigParser(interpolation=None)
        contents.read(desktop_file)
        section = 'Desktop Entry'
        self.assertTrue(section in contents)
        self.assertEqual(contents[section].get('Exec'), 'my-package.app1 %U')
        self.assertEqual(contents[section].get('Icon'), 'app1.png')

        desktop_file = os.path.join(self.meta_dir, 'gui', 'app2.desktop')
        self.assertTrue(os.path.exists(desktop_file),
                        'app2.desktop was not setup correctly')
        contents = configparser.ConfigParser(interpolation=None)
        contents.read(desktop_file)
        section = 'Desktop Entry'
        self.assertTrue(section in contents)
        self.assertEqual(contents[section].get('Exec'), 'my-package.app2 %U')
        self.assertEqual(contents[section].get('Icon'),
                         '${SNAP}/usr/share/app2.png')

    def test_create_meta_with_hook(self):
        hooksdir = os.path.join(self.snap_dir, 'hooks')
        os.makedirs(hooksdir)
        open(os.path.join(hooksdir, 'foo'), 'w').close()
        open(os.path.join(hooksdir, 'bar'), 'w').close()
        os.chmod(os.path.join(hooksdir, 'foo'), 0o755)
        os.chmod(os.path.join(hooksdir, 'bar'), 0o755)
        self.config_data['hooks'] = {
            'foo': {'plugs': ['plug']},
            'bar': {}
        }

        y = self.generate_meta_yaml()

        self.assertThat(
            y, Contains('hooks'), "Expected generated YAML to contain 'hooks'")

        for hook in ('foo', 'bar'):
            generated_hook_path = os.path.join(
                self.prime_dir, 'meta', 'hooks', hook)
            self.assertThat(
                generated_hook_path, FileExists(),
                'The {!r} hook was not setup correctly'.format(hook))

            self.assertThat(
                y['hooks'], Contains(hook),
                'Expected generated hooks to contain {!r}'.format(hook))

        self.assertThat(
            y['hooks']['foo'], Contains('plugs'),
            "Expected generated 'foo' hook to contain 'plugs'")
        self.assertThat(y['hooks']['foo']['plugs'], HasLength(1))
        self.assertThat(y['hooks']['foo']['plugs'][0], Equals('plug'))
        self.assertThat(
            y['hooks']['bar'], Not(Contains('plugs')),
            "Expected generated 'bar' hook to not contain 'plugs'")


class WriteSnapDirectoryTestCase(CreateBaseTestCase):
    def test_write_snap_directory(self):
        # Setup a snap directory containing a few things.
        _create_file(os.path.join(self.snap_dir, 'snapcraft.yaml'))
        _create_file(
            os.path.join(self.snap_dir, 'hooks', 'test-hook'), executable=True)

        # Now write the snap directory, and verify everything was migrated, as
        # well as the hook making it into meta/.
        self.generate_meta_yaml()
        prime_snap_dir = os.path.join(self.prime_dir, 'snap')
        self.assertThat(
            os.path.join(prime_snap_dir, 'snapcraft.yaml'), FileExists())
        self.assertThat(
            os.path.join(prime_snap_dir, 'hooks', 'test-hook'), FileExists())
        self.assertThat(
            os.path.join(self.hooks_dir, 'test-hook'), FileExists())

        # The hook should be empty, because the one in snap/hooks is empty, and
        # no wrapper is generated (i.e. that hook is copied to both locations).
        self.assertThat(
            os.path.join(self.hooks_dir, 'test-hook'), FileContains(''))

    def test_snap_hooks_overwrite_part_hooks(self):
        # Setup a prime/snap directory containing a hook.
        part_hook = os.path.join(self.prime_dir, 'snap', 'hooks', 'test-hook')
        _create_file(part_hook, content='from part', executable=True)

        # Setup a snap directory containing the same hook
        snap_hook = os.path.join(self.snap_dir, 'hooks', 'test-hook')
        _create_file(snap_hook, content='from snap', executable=True)

        # Now write the snap directory, and verify that the snap hook overwrote
        # the part hook in both prime/snap/hooks and prime/meta/hooks.
        self.generate_meta_yaml()
        prime_snap_dir = os.path.join(self.prime_dir, 'snap')
        self.assertThat(
            os.path.join(prime_snap_dir, 'hooks', 'test-hook'), FileExists())
        self.assertThat(
            os.path.join(self.hooks_dir, 'test-hook'), FileExists())

        # Both hooks in snap/hooks and meta/hooks should contain 'from snap' as
        # that one should have overwritten the other (and its wrapper).
        self.assertThat(
            os.path.join(self.prime_dir, 'snap', 'hooks', 'test-hook'),
            FileContains('from snap'))
        self.assertThat(
            os.path.join(self.prime_dir, 'meta', 'hooks', 'test-hook'),
            FileContains('from snap'))

    def test_snap_hooks_not_executable_raises(self):
        # Setup a snap directory containing a few things.
        _create_file(os.path.join(self.snap_dir, 'snapcraft.yaml'))
        _create_file(os.path.join(self.snap_dir, 'hooks', 'test-hook'))

        # Now write the snap directory. This process should fail as the hook
        # isn't executable.
        with testtools.ExpectedException(CommandError,
                                         "hook 'test-hook' is not executable"):
            self.generate_meta_yaml()


class GenerateHookWrappersTestCase(CreateBaseTestCase):
    def test_generate_hook_wrappers(self):
        # Set up the prime directory to contain a few hooks in snap/hooks
        snap_hooks_dir = os.path.join(self.prime_dir, 'snap', 'hooks')
        hook1_path = os.path.join(snap_hooks_dir, 'test-hook1')
        hook2_path = os.path.join(snap_hooks_dir, 'test-hook2')

        for path in (hook1_path, hook2_path):
            _create_file(path, executable=True)

        # Now generate hook wrappers, and verify that they're correct
        self.generate_meta_yaml()
        for hook in ('test-hook1', 'test-hook2'):
            hook_path = os.path.join(self.hooks_dir, hook)
            self.assertThat(hook_path, FileExists())
            self.assertThat(hook_path, tests.IsExecutable())

            # The hook in meta/hooks should exec the one in snap/hooks, as it's
            # a wrapper generated by snapcraft.
            self.assertThat(
                hook_path, FileContains(matcher=Contains(
                    'exec "$SNAP/snap/hooks/{}"'.format(hook))))

    def test_generate_hook_wrappers_not_executable_raises(self):
        # Set up the prime directory to contain a hook in snap/hooks that is
        # not executable.
        snap_hooks_dir = os.path.join(self.prime_dir, 'snap', 'hooks')
        _create_file(os.path.join(snap_hooks_dir, 'test-hook'))

        # Now attempt to generate hook wrappers. This should fail, as the hook
        # itself is not executable.
        with testtools.ExpectedException(CommandError,
                                         "hook 'test-hook' is not executable"):
            self.generate_meta_yaml()


class CreateWithConfinementTestCase(CreateBaseTestCase):

    scenarios = [(confinement, dict(confinement=confinement)) for
                 confinement in ['strict', 'devmode', 'classic']]

    def test_create_meta_with_confinement(self):
        self.config_data['confinement'] = self.confinement

        y = self.generate_meta_yaml()
        self.assertTrue(
            'confinement' in y,
            'Expected "confinement" property to be in snap.yaml')
        self.assertEqual(y['confinement'], self.confinement)


class CreateWithGradeTestCase(CreateBaseTestCase):

    scenarios = [(grade, dict(grade=grade)) for
                 grade in ['stable', 'devel']]

    def test_create_meta_with_grade(self):
        self.config_data['grade'] = self.grade

        y = self.generate_meta_yaml()
        self.assertTrue(
            'grade' in y,
            'Expected "grade" property to be in snap.yaml')
        self.assertEqual(y['grade'], self.grade)


# TODO this needs more tests.
class WrapExeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        # TODO move to use outer interface
        self.packager = _SnapPackaging({'confinement': 'devmode'},
                                       self.prime_dir,
                                       self.parts_dir)

    @patch('snapcraft.internal.common.assemble_env')
    def test_wrap_exe_must_write_wrapper(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(self.parts_dir)

        relative_exe_path = 'test_relexepath'
        open(os.path.join(self.prime_dir, relative_exe_path), 'w').close()

        # Check that the wrapper is created even if there is already a file
        # with the same name.
        open(os.path.join('prime', 'test_relexepath.wrapper'), 'w').close()

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

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
        open(os.path.join(self.prime_dir, relative_exe_path), 'w').close()

        relative_wrapper_path = self.packager._wrap_exe(
            relative_exe_path, basename='new-name')
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

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
        with open(os.path.join(self.prime_dir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = (
            '#!/bin/sh\n'
            '\n\n'
            'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
            'exec "$SNAP/snap_exe"'
            ' "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(self.prime_dir, relative_exe_path), 'r') as exe:
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
        with open(os.path.join(self.prime_dir, relative_exe_path), 'w') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(os.path.join(self.prime_dir, relative_exe_path), 'r') as exe:
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
        path = os.path.join(self.prime_dir, relative_exe_path)
        with open(path, 'wb') as exe:
            exe.write(exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
        with open(path, 'rb') as exe:
            self.assertEqual(exe_contents, exe.read())

    @patch('snapcraft.internal.common.run')
    def test_exe_is_in_path(self, run_mock):
        app_path = os.path.join(self.prime_dir, 'bin', 'app1')
        os.mkdir(os.path.dirname(app_path))
        open(app_path, 'w').close()

        relative_wrapper_path = self.packager._wrap_exe('app1')
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n\n'
                    'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                    'exec "app1" "$@"\n')
        with open(wrapper_path) as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)

    def test_command_does_not_exist(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-does-not-exist'}}

        raised = self.assertRaises(
            EnvironmentError,
            self.packager._wrap_apps, apps)
        self.assertEqual(
            "The specified command 'command-does-not-exist' defined in the "
            "app 'app1' does not exist or is not executable",
            str(raised))

    def test_command_is_not_executable(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-not-executable'}}

        cmd_path = os.path.join(self.prime_dir, 'bin', apps['app1']['command'])
        os.mkdir(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()

        raised = self.assertRaises(
            EnvironmentError,
            self.packager._wrap_apps, apps)
        self.assertEqual(
            "The specified command 'command-not-executable' defined in the "
            "app 'app1' does not exist or is not executable",
            str(raised))

    def test_command_found(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-executable'}}

        cmd_path = os.path.join(self.prime_dir, 'bin', apps['app1']['command'])
        os.mkdir(os.path.dirname(cmd_path))
        open(cmd_path, 'w').close()
        os.chmod(cmd_path, 0o755)

        wrapped_apps = self.packager._wrap_apps(apps)

        self.assertEqual(wrapped_apps,
                         {'app1': {'command': 'command-app1.wrapper'}})


def _create_file(path, *, content='', executable=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)
    if executable:
        os.chmod(path, 0o755)
