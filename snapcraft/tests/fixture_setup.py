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

import contextlib
from functools import partial
import io
import os
import sys
import tempfile
import threading
from textwrap import dedent, fill
from types import ModuleType
import urllib.parse
from unittest import mock
from subprocess import CalledProcessError

import fixtures
import xdg

from snapcraft.tests import fake_servers
from snapcraft.tests.subprocess_utils import (
    call,
    call_with_output,
)


class TempCWD(fixtures.TempDir):

    def setUp(self):
        """Create a temporary directory an cd into it for the test duration."""
        super().setUp()
        current_dir = os.getcwd()
        self.addCleanup(os.chdir, current_dir)
        os.chdir(self.path)


class TempXDG(fixtures.Fixture):
    """Isolate a test from xdg so a private temp config is used."""

    def __init__(self, path):
        super().setUp()
        self.path = path

    def setUp(self):
        super().setUp()
        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_config_home',
            new=os.path.join(self.path, '.config'))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_data_home',
            new=os.path.join(self.path, '.local'))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_cache_home',
            new=os.path.join(self.path, '.cache'))
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher_dirs = mock.patch(
            'xdg.BaseDirectory.xdg_config_dirs',
            new=[xdg.BaseDirectory.xdg_config_home])
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)

        patcher_dirs = mock.patch(
            'xdg.BaseDirectory.xdg_data_dirs',
            new=[xdg.BaseDirectory.xdg_data_home])
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)


class SilentSnapProgress(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.internal.lifecycle.ProgressBar')
        patcher.start()
        self.addCleanup(patcher.stop)


class CleanEnvironment(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        current_environment = os.environ.copy()
        os.environ = {}

        self.addCleanup(os.environ.update, current_environment)


class _FakeStdout(io.StringIO):
    """A fake stdout using StringIO implementing the missing fileno attrib."""

    def fileno(self):
        return 1


class _FakeStderr(io.StringIO):
    """A fake stderr using StringIO implementing the missing fileno attrib."""

    def fileno(self):
        return 2


class _FakeTerminalSize:

    def __init__(self, columns=80):
        self.columns = columns


class FakeTerminal(fixtures.Fixture):

    def __init__(self, columns=80, isatty=True):
        self.columns = columns
        self.isatty = isatty

    def _setUp(self):
        patcher = mock.patch('shutil.get_terminal_size')
        mock_terminal_size = patcher.start()
        mock_terminal_size.return_value = _FakeTerminalSize(self.columns)
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout', new_callable=_FakeStdout)
        self.mock_stdout = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stderr', new_callable=_FakeStderr)
        self.mock_stderr = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('os.isatty')
        mock_isatty = patcher.start()
        mock_isatty.return_value = self.isatty
        self.addCleanup(patcher.stop)

    def getvalue(self, stderr=False):
        if stderr:
            return self.mock_stderr.getvalue()
        else:
            return self.mock_stdout.getvalue()


class FakePartsWiki(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        self.fake_parts_wiki_fixture = FakePartsWikiRunning()
        self.useFixture(self.fake_parts_wiki_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))


class FakePartsWikiWithSlashes(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        self.fake_parts_wiki_with_slashes_fixture = (
            FakePartsWikiWithSlashesRunning())
        self.useFixture(self.fake_parts_wiki_with_slashes_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))


class FakePartsWikiOrigin(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        self.fake_parts_wiki_origin_fixture = FakePartsWikiOriginRunning()
        self.useFixture(self.fake_parts_wiki_origin_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))


class FakeParts(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        self.fake_parts_server_fixture = FakePartsServerRunning()
        self.useFixture(self.fake_parts_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_PARTS_URI',
            urllib.parse.urljoin(
                self.fake_parts_server_fixture.url, 'parts.yaml')))
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))


class FakeStore(fixtures.Fixture):

    def setUp(self):
        super().setUp()
        # In case the variable was not set or it was empty.
        self.useFixture(fixtures.EnvironmentVariable('TEST_STORE', 'fake'))

        self.needs_refresh = False

        self.fake_sso_server_fixture = FakeSSOServerRunning(self)
        self.useFixture(self.fake_sso_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL',
            urllib.parse.urljoin(
                self.fake_sso_server_fixture.url, 'api/v2/')))

        self.useFixture(fixtures.EnvironmentVariable(
            'STORE_RETRIES', '1'))
        self.useFixture(fixtures.EnvironmentVariable(
            'STORE_BACKOFF', '0'))

        self.fake_store_upload_server_fixture = FakeStoreUploadServerRunning()
        self.useFixture(self.fake_store_upload_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            self.fake_store_upload_server_fixture.url))

        self.fake_store_api_server_fixture = FakeStoreAPIServerRunning(self)
        self.useFixture(self.fake_store_api_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_API_ROOT_URL',
            urllib.parse.urljoin(
                self.fake_store_api_server_fixture.url, 'dev/api/')))

        self.fake_store_search_server_fixture = FakeStoreSearchServerRunning()
        self.useFixture(self.fake_store_search_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_SEARCH_ROOT_URL',
            self.fake_store_search_server_fixture.url))

        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))


class _FakeServerRunning(fixtures.Fixture):

    # To be defined by child fixtures.
    fake_server = None

    def setUp(self):
        super().setUp()
        self._start_fake_server()

    def _start_fake_server(self):
        server_address = ('', 0)
        self.server = self.fake_server(server_address)
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_fake_server, server_thread)
        self.url = 'http://localhost:{}/'.format(self.server.server_port)

    def _stop_fake_server(self, thread):
        self.server.shutdown()
        self.server.socket.close()
        thread.join()


class FakePartsWikiOriginRunning(_FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiOriginServer


class FakePartsWikiRunning(_FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiServer


class FakePartsWikiWithSlashesRunning(_FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiWithSlashesServer


class FakePartsServerRunning(_FakeServerRunning):

    fake_server = fake_servers.FakePartsServer


class FakeSSOServerRunning(_FakeServerRunning):

    def __init__(self, fake_store):
        super().__init__()
        self.fake_server = partial(fake_servers.FakeSSOServer, fake_store)


class FakeStoreUploadServerRunning(_FakeServerRunning):

    fake_server = fake_servers.FakeStoreUploadServer


class FakeStoreAPIServerRunning(_FakeServerRunning):

    def __init__(self, fake_store):
        super().__init__()
        self.fake_server = partial(fake_servers.FakeStoreAPIServer, fake_store)


class FakeStoreSearchServerRunning(_FakeServerRunning):

    fake_server = fake_servers.FakeStoreSearchServer


class StagingStore(fixtures.Fixture):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_API_ROOT_URL',
            'https://myapps.developer.staging.ubuntu.com/dev/api/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            'https://upload.apps.staging.ubuntu.com/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL',
            'https://login.staging.ubuntu.com/api/v2/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_SEARCH_ROOT_URL',
            'https://search.apps.staging.ubuntu.com/'))


class TestStore(fixtures.Fixture):

    def setUp(self):
        super().setUp()
        test_store = os.getenv('TEST_STORE') or 'fake'
        if test_store == 'fake':
            self.useFixture(FakeStore())
            self.register_count_limit = 10
            self.reserved_snap_name = 'test-reserved-snap-name'
            self.already_owned_snap_name = 'test-already-owned-snap-name'
        elif test_store == 'staging':
            self.useFixture(StagingStore())
            self.register_count_limit = 10
            self.reserved_snap_name = 'bash'
        elif test_store == 'production':
            # Use the default server URLs
            self.register_count_limit = 10
            self.reserved_snap_name = 'bash'
        else:
            raise ValueError(
                'Unknown test store option: {}'.format(test_store))

        self.user_email = os.getenv(
            'TEST_USER_EMAIL', 'u1test+snapcraft@canonical.com')
        self.test_track_snap = os.getenv(
            'TEST_SNAP_WITH_TRACKS', 'test-snapcraft-tracks')
        if test_store == 'fake':
            self.user_password = 'test correct password'
        else:
            self.user_password = os.getenv('TEST_USER_PASSWORD')


class FakePlugin(fixtures.Fixture):
    '''Dynamically generate a new module containing the provided plugin'''

    def __init__(self, plugin_name, plugin_class):
        super().__init__()
        self._import_name = 'snapcraft.plugins.{}'.format(
            plugin_name.replace('-', '_'))
        self._plugin_class = plugin_class

    def _setUp(self):
        plugin_module = ModuleType(self._import_name)
        setattr(plugin_module, self._plugin_class.__name__, self._plugin_class)
        sys.modules[self._import_name] = plugin_module
        self.addCleanup(self._remove_module)

    def _remove_module(self):
        del sys.modules[self._import_name]


def check_output_side_effect(fail_on_remote=False, fail_on_default=False):
    def call_effect(*args, **kwargs):
        if args[0] == ['lxc', 'remote', 'get-default']:
            if fail_on_default:
                raise CalledProcessError(returncode=255, cmd=args[0])
            else:
                return 'local'.encode('utf-8')
        elif args[0] == ['lxc', 'list', 'my-remote:'] and fail_on_remote:
            raise CalledProcessError(returncode=255, cmd=args[0])
        elif args[0][:3] == ['lxc', 'list', '--format=json']:
            return '''
                [{"name": "snapcraft-snap-test",
                  "status": "Stopped",
                  "devices": {"build-snap-test":[]}}]
                '''.encode('utf-8')
        else:
            return ''.encode('utf-8')
    return call_effect


class FakeAptGetBuildDep(fixtures.Fixture):
    '''Mock apt-get build-dep output'''

    _PROLOG = dedent('''
        NOTE: This is only a simulation!
              apt-get needs root privileges for real execution.
              Keep also in mind that locking is deactivated,
              so don't depend on the relevance to the real current situation!
        Note, using file '{}' to get the build dependencies
        Reading package lists...
        Building dependency tree...
        Reading state information...''')
    _PROBLEMS = dedent('''
        Some packages could not be installed. This may mean that you have
        requested an impossible situation or if you are using the unstable
        distribution that some required packages have not yet been created
        or been moved out of Incoming.
        The following information may help to resolve the situation:

        The following packages have unmet dependencies:
         builddeps:{}{} : {}
        E: Unable to correct problems, you have held broken packages.''')
    _NEW = dedent('''
        The following NEW packages will be installed:
          {}
        3 upgraded, {} newly installed, 0 to remove and 5 not upgraded.''')

    def __init__(self, packages, arch='', update_error=False,
                 not_cached=False, not_available=False):
        self.filename = '{}/{}abcdef.dsc'.format(tempfile.gettempdir(),
                                                 tempfile.gettempprefix())
        if arch:
            arch = ':{}'.format(arch)
        if not_cached:
            note = 'Depends: {}{} but it is not going to be installed'
        elif not_available:
            note = 'Depends: {}{} but it is not installable'
        else:
            note = '{}{}'
        errors = []
        for package in packages:
            errors.append(note.format(package, arch))
        if not_cached or not_available:
            self.exception = True
            details = self._PROBLEMS.format(
                self.filename, arch, '\n'.join(errors))
        else:
            self.exception = False
            details = self._NEW.format(fill(' '.join(errors),
                                            subsequent_indent='  ',
                                            break_on_hyphens=False),
                                       len(errors))
        self.output = '{}\n{}'.format(self._PROLOG.format(self.filename),
                                      details).encode(
                                          sys.getfilesystemencoding())
        self.update_error = update_error

    def _setUp(self):
        patcher = mock.patch('tempfile.NamedTemporaryFile')
        tempfile_mock = patcher.start()
        tempfile_mock.return_value.__enter__.return_value.name = self.filename
        self.addCleanup(patcher.stop)

        patcher = mock.patch('subprocess.check_output')
        self.check_output_mock = patcher.start()
        self.check_output_mock.side_effect = self.check_output_side_effect()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.repo._deb.open',
                             mock.mock_open())
        self.open_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def check_output_side_effect(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ['apt-get', 'build-dep']:
                if self.exception:
                    raise CalledProcessError(100, args[0], self.output)
                else:
                    return self.output
            if args[0][:3] == ['sudo', 'apt-get', 'update']:
                server = 'http://archive.ubuntu.com/ubuntu'
                template = '{}:{} {} xenial InRelease\n'
                output = template.format('Get', '9', server)
                if self.update_error:
                    output += template.format('Err', '9', server)
                return output.encode(sys.getfilesystemencoding())
            elif args[0][:2] == ['dpkg', '--print-foreign-architectures']:
                return 'amd64\n'.encode(sys.getfilesystemencoding())
        return call_effect


class FakeLXD(fixtures.Fixture):
    '''...'''

    def __init__(self, fail_on_remote=False, fail_on_default=False):
        self.fail_on_remote = fail_on_remote
        self.fail_on_default = fail_on_default

    def _setUp(self):
        patcher = mock.patch('snapcraft.internal.lxd.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.lxd.check_output')
        self.check_output_mock = patcher.start()
        self.check_output_mock.side_effect = check_output_side_effect(
            self.fail_on_remote, self.fail_on_default)
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.lxd.sleep', lambda _: None)
        patcher.start()
        self.addCleanup(patcher.stop)


class GitRepo(fixtures.Fixture):
    '''Create a git repo in the current directory'''

    def setUp(self):
        super().setUp()
        name = 'git-source'  # must match what the tests expect

        def _add_and_commit_file(path, filename, contents=None, message=None):
            if not contents:
                contents = filename
            if not message:
                message = filename

            with open(os.path.join(path, filename), 'w') as fp:
                fp.write(contents)

            call(['git', '-C', name, 'add', filename])
            call(['git', '-C', name, 'commit', '-am', message])

        os.makedirs(name)
        call(['git', '-C', name, 'init'])
        call(['git', '-C', name, 'config',
              'user.name', 'Test User'])
        call(['git', '-C', name, 'config',
              'user.email', 'testuser@example.com'])

        _add_and_commit_file(name, 'testing')
        call(['git', '-C', name, 'branch', 'test-branch'])

        _add_and_commit_file(name, 'testing-2')
        call(['git', '-C', name, 'tag', 'feature-tag'])

        _add_and_commit_file(name, 'testing-3')

        self.commit = call_with_output(
            ['git', '-C', name, 'rev-parse', 'HEAD'])


@contextlib.contextmanager
def return_to_cwd():
    cwd = os.getcwd()
    try:
        yield
    finally:
        os.chdir(cwd)


class BzrRepo(fixtures.Fixture):

    def __init__(self, name):
        self.name = name

    def setUp(self):
        super().setUp()

        with return_to_cwd():
            os.makedirs(self.name)
            os.chdir(self.name)
            call(['bzr', 'init'])
            call(['bzr', 'whoami', 'Test User <test.user@example.com>'])
            with open('testing', 'w') as fp:
                fp.write('testing')

            call(['bzr', 'add', 'testing'])
            call(['bzr', 'commit', '-m', 'testing'])
            call(['bzr', 'tag', 'feature-tag'])
            revno = call_with_output(['bzr', 'revno'])

            self.commit = revno


class SvnRepo(fixtures.Fixture):

    def __init__(self, name):
        self.name = name

    def setUp(self):
        super().setUp()

        working_tree = 'svn-repo'
        call(['svnadmin', 'create', self.name])
        call(['svn', 'checkout',
              'file://{}'.format(os.path.join(os.getcwd(), self.name)),
              working_tree])

        with return_to_cwd():
            os.chdir(working_tree)
            with open('testing', 'w') as fp:
                fp.write('testing')

            call(['svn', 'add', 'testing'])
            call(['svn', 'commit', '-m', 'svn testing'])
            revno = '1'

            self.commit = revno


class HgRepo(fixtures.Fixture):

    def __init__(self, name):
        self.name = name

    def setUp(self):
        super().setUp()

        with return_to_cwd():
            os.makedirs(self.name)
            os.chdir(self.name)
            call(['hg', 'init'])
            with open('testing', 'w') as fp:
                fp.write('testing')

            call(['hg', 'add', 'testing'])
            call(['hg', 'commit', '-m', 'testing',
                  '-u', 'Test User <test.user@example.com>'])
            call(['hg', 'tag', 'feature-tag',
                  '-u', 'Test User <test.user@example.com>'])
            revno = call_with_output(['hg', 'id']).split()[0]

            self.commit = revno
