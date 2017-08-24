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
from snapcraft.internal import errors
from snapcraft import ProjectOptions, tests


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
            'environment': {
                'GLOBAL': 'y',
            }
        }

        patcher = patch(
            'snapcraft.internal.project_loader.get_snapcraft_yaml')
        self.mock_get_yaml = patcher.start()
        self.mock_get_yaml.return_value = os.path.join(
            'snap', 'snapcraft.yaml')
        self.addCleanup(patcher.stop)

        # Ensure the ensure snapcraft.yaml method has something to copy.
        _create_file(os.path.join('snap', 'snapcraft.yaml'))

        self.meta_dir = os.path.join(self.prime_dir, 'meta')
        self.hooks_dir = os.path.join(self.meta_dir, 'hooks')
        self.snap_yaml = os.path.join(self.meta_dir, 'snap.yaml')

        self.project_options = ProjectOptions()

    def generate_meta_yaml(self):
        create_snap_packaging(self.config_data, self.project_options, 'dummy')

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
                    'environment': {'GLOBAL': 'y'},
                    'summary': 'my summary',
                    'name': 'my-package',
                    'version': '1.0'}

        self.assertThat(y, Equals(expected))

    def test_create_meta_with_epoch(self):
        self.config_data['epoch'] = '1*'

        y = self.generate_meta_yaml()
        self.assertTrue(
            'epoch' in y,
            'Expected "epoch" property to be copied into snap.yaml')
        self.assertThat(y['epoch'], Equals('1*'))

    def test_create_meta_with_assumes(self):
        self.config_data['assumes'] = ['feature1', 'feature2']

        y = self.generate_meta_yaml()
        self.assertTrue(
            'assumes' in y,
            'Expected "assumes" property to be copied into snap.yaml')
        self.assertThat(y['assumes'], Equals(['feature1', 'feature2']))

    def test_create_gadget_meta_with_gadget_yaml(self):
        gadget_yaml = 'stub entry: stub value'
        _create_file('gadget.yaml', content=gadget_yaml)

        self.config_data['type'] = 'gadget'
        create_snap_packaging(self.config_data, self.project_options, 'dummy')

        expected_gadget = os.path.join(self.meta_dir, 'gadget.yaml')
        self.assertTrue(os.path.exists(expected_gadget))

        self.assertThat(expected_gadget, FileContains(gadget_yaml))

    def test_create_gadget_meta_with_missing_gadget_yaml_raises_error(self):
        self.config_data['type'] = 'gadget'

        self.assertRaises(
            errors.MissingGadgetError,
            create_snap_packaging,
            self.config_data,
            self.project_options,
            'dummy'
        )

    def test_create_meta_with_declared_icon(self):
        _create_file(os.path.join(os.curdir, 'my-icon.png'))
        self.config_data['icon'] = 'my-icon.png'

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, 'gui', 'icon.png'),
                        FileExists())

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon_with_dots(self):
        _create_file('com.my.icon.png')
        self.config_data['icon'] = 'com.my.icon.png'

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, 'gui', 'icon.png'),
                        FileExists())

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon_in_parent_dir(self):
        _create_file('my-icon.png')
        builddir = os.path.join(os.curdir, 'subdir')
        os.mkdir(builddir)
        os.chdir(builddir)
        self.config_data['icon'] = '../my-icon.png'

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, 'gui', 'icon.png'),
                        FileExists())

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_create_meta_with_declared_icon_and_setup(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        setup_icon_content = 'setup icon'
        _create_file(os.path.join(gui_path, 'icon.png'),
                     content=setup_icon_content)

        declared_icon_content = 'declared icon'
        _create_file('my-icon.png',
                     content=declared_icon_content)
        self.config_data['icon'] = 'my-icon.png'

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertTrue(os.path.exists(expected_icon),
                        'icon.png was not setup correctly')
        self.assertThat(expected_icon, FileContains(declared_icon_content))

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

        # Check for the correct deprecation message.
        self.assertIn(
            "Assets in 'setup/gui' should now be placed in 'snap/gui'.",
            fake_logger.output)
        self.assertIn(
            "See http://snapcraft.io/docs/deprecation-notices/dn3",
            fake_logger.output)

    def test_create_meta_with_declared_icon_and_setup_ran_twice_ok(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = 'setup icon'
        _create_file(os.path.join(gui_path, 'icon.png'), content=icon_content)

        _create_file('my-icon.png')
        self.config_data['icon'] = 'my-icon.png'

        create_snap_packaging(self.config_data, self.project_options, 'dummy')

        # Running again should be good
        create_snap_packaging(self.config_data, self.project_options, 'dummy')

    def test_create_meta_with_icon_in_setup(self):
        gui_path = os.path.join('setup', 'gui')
        os.makedirs(gui_path)
        icon_content = 'setup icon'
        _create_file(os.path.join(gui_path, 'icon.png'), content=icon_content)

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, 'gui', 'icon.png')
        self.assertThat(expected_icon, FileContains(icon_content))

        self.assertFalse('icon' in y,
                         'icon found in snap.yaml {}'.format(y))

    def test_version_script(self):
        self.config_data['version-script'] = 'echo 10.1-devel'

        y = self.generate_meta_yaml()

        self.assertThat(y['version'], Equals('10.1-devel'))

    def test_version_script_exits_bad(self):
        self.config_data['version-script'] = 'exit 1'

        with testtools.ExpectedException(CommandError):
            self.generate_meta_yaml()

    def test_version_script_with_no_output(self):
        self.config_data['version-script'] = 'echo'

        with testtools.ExpectedException(CommandError):
            self.generate_meta_yaml()

    def test_create_meta_with_app(self):
        os.mkdir(self.prime_dir)
        _create_file(os.path.join(self.prime_dir, 'app.sh'))
        self.config_data['apps'] = {
            'app1': {'command': 'app.sh'},
            'app2': {'command': 'app.sh', 'plugs': ['network']},
            'app3': {'command': 'app.sh', 'plugs': ['network-server']},
            'app4': {'command': 'app.sh', 'plugs': ['network-server'],
                     'environment': {'XDG_SOMETHING': '$SNAP_USER_DATA',
                                     'LANG': 'C'}},
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
                'app4': {
                    'command': 'command-app4.wrapper',
                    'plugs': ['network-server'],
                    'environment': {
                        'XDG_SOMETHING': '$SNAP_USER_DATA',
                        'LANG': 'C'}
                },
            },
            'description': 'my description',
            'summary': 'my summary',
            'name': 'my-package',
            'version': '1.0',
            'confinement': 'devmode',
            'environment': {'GLOBAL': 'y'},
            'plugs': {
                'network-server': {
                    'interface': 'network-bind',
                }
            }
        }

        self.assertThat(y, Equals(expected))

    def test_create_meta_with_app_desktop_key(self):
        os.mkdir(self.prime_dir)
        _create_file(os.path.join(self.prime_dir, 'app.sh'))
        _create_file(os.path.join(self.prime_dir, 'app1.desktop'),
                     content='[Desktop Entry]\nExec=app1.exe\nIcon=app1.png')
        icon_dir = os.path.join(self.prime_dir, 'usr', 'share')
        os.makedirs(icon_dir)
        _create_file(os.path.join(icon_dir, 'app2.png'))
        _create_file(os.path.join(self.prime_dir, 'app2.desktop'),
                     content='[Desktop Entry]\nExec=app2.exe\nIcon=/usr/share/'
                             'app2.png')
        _create_file(os.path.join(self.prime_dir, 'app3.desktop'),
                     content='[Desktop Entry]\nExec=app3.exe\nIcon=app3.png')
        self.config_data['apps'] = {
            'app1': {'command': 'app.sh', 'desktop': 'app1.desktop'},
            'app2': {'command': 'app.sh', 'desktop': 'app2.desktop'},
            'my-package': {'command': 'app.sh', 'desktop': 'app3.desktop'}
        }

        self.generate_meta_yaml()

        desktop_file = os.path.join(self.meta_dir, 'gui', 'app1.desktop')
        self.assertThat(desktop_file, FileExists())
        contents = configparser.ConfigParser(interpolation=None)
        contents.read(desktop_file)
        section = 'Desktop Entry'
        self.assertTrue(section in contents)
        self.assertThat(
            contents[section].get('Exec'), Equals('my-package.app1 %U'))
        self.assertThat(contents[section].get('Icon'), Equals('app1.png'))

        desktop_file = os.path.join(self.meta_dir, 'gui', 'app2.desktop')
        self.assertThat(desktop_file, FileExists())
        contents = configparser.ConfigParser(interpolation=None)
        contents.read(desktop_file)
        section = 'Desktop Entry'
        self.assertTrue(section in contents)
        self.assertThat(
            contents[section].get('Exec'), Equals('my-package.app2 %U'))
        self.assertThat(contents[section].get('Icon'),
                        Equals('${SNAP}/usr/share/app2.png'))

        desktop_file = os.path.join(self.meta_dir, 'gui', 'my-package.desktop')
        self.assertThat(desktop_file, FileExists())
        contents = configparser.ConfigParser(interpolation=None)
        contents.read(desktop_file)
        section = 'Desktop Entry'
        self.assertTrue(section in contents)
        self.assertThat(contents[section].get('Exec'), Equals('my-package %U'))

        snap_yaml = os.path.join('prime', 'meta', 'snap.yaml')
        self.assertThat(snap_yaml, Not(FileContains('desktop: app1.desktop')))
        self.assertThat(snap_yaml, Not(FileContains('desktop: app2.desktop')))
        self.assertThat(snap_yaml, Not(FileContains('desktop: app3.desktop')))
        self.assertThat(snap_yaml,
                        Not(FileContains('desktop: my-package.desktop')))

    def test_create_meta_with_hook(self):
        hooksdir = os.path.join(self.snap_dir, 'hooks')
        os.makedirs(hooksdir)
        _create_file(os.path.join(hooksdir, 'foo'), executable=True)
        _create_file(os.path.join(hooksdir, 'bar'), executable=True)
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
        self.assertThat(y['confinement'], Equals(self.confinement))


class EnsureFilePathsTestCase(CreateBaseTestCase):

    scenarios = [
        ('desktop', dict(
            filepath='usr/share/dekstop/desktop.desktop',
            content='[Desktop Entry]\nExec=app2.exe\nIcon=/usr/share/app2.png',
            key='desktop')),
        ('completer', dict(
            filepath='usr/share/completions/complete.sh',
            content='#/bin/bash\n',
            key='completer')),
    ]

    def test_file_path_entry(self):
        self.config_data['apps'] = {'app': {self.key: self.filepath}}
        _create_file(os.path.join('prime', self.filepath),
                     content=self.content)

        # If the path exists this should not fail
        self.generate_meta_yaml()


class EnsureFilePathsTestCaseFails(CreateBaseTestCase):

    scenarios = [
        ('desktop', dict(
            filepath='usr/share/dekstop/desktop.desktop',
            key='desktop')),
        ('completer', dict(
            filepath='usr/share/completions/complete.sh',
            key='completer')),
    ]

    def test_file_path_entry(self):
        self.config_data['apps'] = {'app': {self.key: self.filepath}}

        self.assertRaises(
            errors.SnapcraftPathEntryError, self.generate_meta_yaml)


class CreateWithGradeTestCase(CreateBaseTestCase):

    scenarios = [(grade, dict(grade=grade)) for
                 grade in ['stable', 'devel']]

    def test_create_meta_with_grade(self):
        self.config_data['grade'] = self.grade

        y = self.generate_meta_yaml()
        self.assertTrue(
            'grade' in y,
            'Expected "grade" property to be in snap.yaml')
        self.assertThat(y['grade'], Equals(self.grade))


# TODO this needs more tests.
class WrapExeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        # TODO move to use outer interface
        self.packager = _SnapPackaging(
            {'confinement': 'devmode'},
            ProjectOptions(),
            'dummy'
        )

    @patch('snapcraft.internal.common.assemble_env')
    def test_wrap_exe_must_write_wrapper(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(self.parts_dir)

        relative_exe_path = 'test_relexepath'
        _create_file(os.path.join(self.prime_dir, relative_exe_path))

        # Check that the wrapper is created even if there is already a file
        # with the same name.
        _create_file(os.path.join(self.prime_dir, 'test_relexepath.wrapper'))

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n\n'
                    'export LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:'
                    '$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')

        self.assertThat(wrapper_path, FileContains(expected))

    @patch('snapcraft.internal.common.assemble_env')
    def test_wrap_exe_writes_wrapper_with_basename(self, mock_assemble_env):
        mock_assemble_env.return_value = """\
PATH={0}/part1/install/usr/bin:{0}/part1/install/bin
""".format(self.parts_dir)

        relative_exe_path = 'test_relexepath'
        _create_file(os.path.join(self.prime_dir, relative_exe_path))

        relative_wrapper_path = self.packager._wrap_exe(
            relative_exe_path, basename='new-name')
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        self.assertThat(relative_wrapper_path, Equals('new-name.wrapper'))

        expected = ('#!/bin/sh\n'
                    'PATH=$SNAP/usr/bin:$SNAP/bin\n\n'
                    'export LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:'
                    '$LD_LIBRARY_PATH\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        self.assertThat(wrapper_path, FileContains(expected))

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
        _create_file(os.path.join(self.prime_dir, relative_exe_path),
                     content=exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = (
            '#!/bin/sh\n'
            'exec "$SNAP/snap_exe" "$SNAP/test_relexepath" "$@"\n')
        self.assertThat(wrapper_path, FileContains(expected))

        # The shebang wasn't changed, since we don't know what the
        # path will be on the installed system.
        self.assertThat(os.path.join(self.prime_dir, relative_exe_path),
                        FileContains(exe_contents))

    def test_non_snap_shebangs_ignored(self):
        """Shebangs not pointing to the snap's install dir are ignored.

        If the shebang points to a system executable, there's no need to
        interfere.
        """
        relative_exe_path = 'test_relexepath'
        exe_contents = '#!/bin/bash\necho hello\n'
        _create_file(os.path.join(self.prime_dir, relative_exe_path),
                     content=exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        self.assertThat(wrapper_path, FileContains(expected))

        self.assertThat(os.path.join(self.prime_dir, relative_exe_path),
                        FileContains(exe_contents))

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
        _create_file(path, content=exe_contents)

        relative_wrapper_path = self.packager._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'exec "$SNAP/test_relexepath" "$@"\n')
        self.assertThat(wrapper_path, FileContains(expected))

        with open(path, 'rb') as exe:
            self.assertThat(exe.read(), Equals(exe_contents))

    @patch('snapcraft.internal.common.run_output')
    def test_exe_is_in_path(self, run_mock):
        app_path = os.path.join(self.prime_dir, 'bin', 'app1')
        _create_file(app_path)

        relative_wrapper_path = self.packager._wrap_exe('app1')
        wrapper_path = os.path.join(self.prime_dir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    'exec "app1" "$@"\n')
        self.assertThat(wrapper_path, FileContains(expected))

    def test_command_does_not_exist(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-does-not-exist'}}

        raised = self.assertRaises(
            errors.InvalidAppCommandError,
            self.packager._wrap_apps, apps)
        self.assertThat(
            str(raised),
            Equals(
                "The specified command 'command-does-not-exist' defined in "
                "the app 'app1' does not exist or is not executable"))

    def test_command_is_not_executable(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-not-executable'}}

        cmd_path = os.path.join(self.prime_dir, 'bin', apps['app1']['command'])
        _create_file(cmd_path)

        raised = self.assertRaises(
            errors.InvalidAppCommandError,
            self.packager._wrap_apps, apps)
        self.assertThat(
            str(raised),
            Equals("The specified command 'command-not-executable' defined in "
                   "the app 'app1' does not exist or is not executable"))

    def test_command_found(self):
        common.env = ['PATH={}/bin:$PATH'.format(self.prime_dir)]

        apps = {'app1': {'command': 'command-executable'}}

        cmd_path = os.path.join(self.prime_dir, 'bin', apps['app1']['command'])
        _create_file(cmd_path, executable=True)

        wrapped_apps = self.packager._wrap_apps(apps)

        self.assertThat(wrapped_apps,
                        Equals({'app1': {'command': 'command-app1.wrapper'}}))


def _create_file(path, *, content='', executable=False):
    basepath = os.path.dirname(path)
    if basepath:
        os.makedirs(basepath, exist_ok=True)
    mode = 'wb' if type(content) == bytes else 'w'
    with open(path, mode) as f:
        f.write(content)
    if executable:
        os.chmod(path, 0o755)
