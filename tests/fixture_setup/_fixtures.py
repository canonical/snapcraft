# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import copy
import io
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import threading
import urllib.parse
from functools import partial
from unittest import mock

import fixtures
import xdg

from tests import fake_servers
from tests.fake_servers import api, search, upload
from tests.subprocess_utils import call, call_with_output

# we do not want snapcraft imports for the integration tests
try:
    from snapcraft import yaml_utils
except ImportError:
    import yaml as yaml_utils  # type: ignore


class TempCWD(fixtures.Fixture):
    def __init__(self, rootdir=None):
        super().__init__()
        if rootdir is None and "TMPDIR" in os.environ:
            rootdir = os.environ.get("TMPDIR")
        self.rootdir = rootdir
        self._data_path = os.getenv("SNAPCRAFT_TEST_KEEP_DATA_PATH", None)

    def setUp(self):
        """Create a temporary directory an cd into it for the test duration."""
        super().setUp()
        if self._data_path:
            os.makedirs(self._data_path)
            self.path = self._data_path
        else:
            self.path = tempfile.mkdtemp(dir=self.rootdir)
        current_dir = os.getcwd()
        self.addCleanup(os.chdir, current_dir)
        if not self._data_path:
            self.addCleanup(shutil.rmtree, self.path, ignore_errors=True)
        os.chdir(self.path)


class TempXDG(fixtures.Fixture):
    """Isolate a test from xdg so a private temp config is used."""

    def __init__(self, path):
        super().setUp()
        self.path = path

    def setUp(self):
        super().setUp()
        patcher = mock.patch(
            "xdg.BaseDirectory.xdg_config_home", new=os.path.join(self.path, ".config")
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            "xdg.BaseDirectory.xdg_data_home", new=os.path.join(self.path, ".local")
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            "xdg.BaseDirectory.xdg_cache_home", new=os.path.join(self.path, ".cache")
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher_dirs = mock.patch(
            "xdg.BaseDirectory.xdg_config_dirs", new=[xdg.BaseDirectory.xdg_config_home]
        )
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)

        patcher_dirs = mock.patch(
            "xdg.BaseDirectory.xdg_data_dirs", new=[xdg.BaseDirectory.xdg_data_home]
        )
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)

        self.useFixture(
            fixtures.EnvironmentVariable(
                "XDG_CONFIG_HOME", os.path.join(self.path, ".config")
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "XDG_DATA_HOME", os.path.join(self.path, ".local")
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "XDG_CACHE_HOME", os.path.join(self.path, ".cache")
            )
        )


class CleanEnvironment(fixtures.Fixture):
    def setUp(self):
        super().setUp()

        current_environment = copy.deepcopy(os.environ)
        os.environ.clear()

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
        patcher = mock.patch("shutil.get_terminal_size")
        mock_terminal_size = patcher.start()
        mock_terminal_size.return_value = _FakeTerminalSize(self.columns)
        self.addCleanup(patcher.stop)

        patcher = mock.patch("sys.stdout", new_callable=_FakeStdout)
        self.mock_stdout = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("sys.stderr", new_callable=_FakeStderr)
        self.mock_stderr = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("os.isatty")
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
        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))


class FakePartsWikiWithSlashes(fixtures.Fixture):
    def setUp(self):
        super().setUp()

        self.fake_parts_wiki_with_slashes_fixture = FakePartsWikiWithSlashesRunning()
        self.useFixture(self.fake_parts_wiki_with_slashes_fixture)
        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))


class FakePartsWikiOrigin(fixtures.Fixture):
    def setUp(self):
        super().setUp()

        self.fake_parts_wiki_origin_fixture = FakePartsWikiOriginRunning()
        self.useFixture(self.fake_parts_wiki_origin_fixture)
        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))


class FakeParts(fixtures.Fixture):
    def setUp(self):
        super().setUp()

        self.fake_parts_server_fixture = FakePartsServerRunning()
        self.useFixture(self.fake_parts_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_PARTS_URI",
                urllib.parse.urljoin(self.fake_parts_server_fixture.url, "parts.yaml"),
            )
        )
        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))


class FakeStore(fixtures.Fixture):
    def setUp(self):
        super().setUp()
        # In case the variable was not set or it was empty.
        self.useFixture(fixtures.EnvironmentVariable("TEST_STORE", "fake"))

        self.needs_refresh = False

        self.fake_sso_server_fixture = FakeSSOServerRunning(self)
        self.useFixture(self.fake_sso_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_ONE_SSO_URL", self.fake_sso_server_fixture.url
            )
        )

        self.useFixture(fixtures.EnvironmentVariable("STORE_RETRIES", "1"))
        self.useFixture(fixtures.EnvironmentVariable("STORE_BACKOFF", "0"))

        self.fake_store_upload_server_fixture = FakeStoreUploadServerRunning()
        self.useFixture(self.fake_store_upload_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_UPLOAD_URL", self.fake_store_upload_server_fixture.url,
            )
        )

        self.fake_store_api_server_fixture = FakeStoreAPIServerRunning(self)
        self.useFixture(self.fake_store_api_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_DASHBOARD_URL", self.fake_store_api_server_fixture.url
            )
        )

        self.fake_store_search_server_fixture = FakeStoreSearchServerRunning()
        self.useFixture(self.fake_store_search_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_API_URL", self.fake_store_search_server_fixture.url,
            )
        )

        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))


class FakeServerRunning(fixtures.Fixture):
    # fake_server needs to be set by implementing classes

    def setUp(self):
        super().setUp()
        self._start_fake_server()

    def _start_fake_server(self):
        server_address = ("", 0)
        self.server = self.fake_server(server_address)
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_fake_server, server_thread)
        self.url = "http://localhost:{}/".format(self.server.server_port)

    def _stop_fake_server(self, thread):
        self.server.shutdown()
        self.server.socket.close()
        thread.join()


class FakePartsWikiOriginRunning(FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiOriginServer


class FakePartsWikiRunning(FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiServer


class FakePartsWikiWithSlashesRunning(FakeServerRunning):

    fake_server = fake_servers.FakePartsWikiWithSlashesServer


class FakePartsServerRunning(FakeServerRunning):

    fake_server = fake_servers.FakePartsServer


class FakeSSOServerRunning(FakeServerRunning):
    def __init__(self, fake_store):
        super().__init__()
        self.fake_server = partial(fake_servers.FakeSSOServer, fake_store)


class FakeStoreUploadServerRunning(FakeServerRunning):

    fake_server = upload.FakeStoreUploadServer


class FakeStoreAPIServerRunning(FakeServerRunning):
    def __init__(self, fake_store):
        super().__init__()
        self.fake_server = partial(api.FakeStoreAPIServer, fake_store)


class FakeStoreSearchServerRunning(FakeServerRunning):

    fake_server = search.FakeStoreSearchServer


class StagingStore(fixtures.Fixture):
    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_DASHBOARD_URL", "https://dashboard.staging.snapcraft.io/",
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_UPLOAD_URL", "https://upload.apps.staging.ubuntu.com/",
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_ONE_SSO_URL", "https://login.staging.ubuntu.com/"
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "STORE_API_URL", "https://api.staging.snapcraft.io/"
            )
        )


class TestStore(fixtures.Fixture):
    def setUp(self):
        super().setUp()
        test_store = os.getenv("TEST_STORE") or "fake"
        if test_store == "fake":
            self.useFixture(FakeStore())
            self.register_count_limit = 10
            self.reserved_snap_name = "test-reserved-snap-name"
            self.already_owned_snap_name = "test-already-owned-snap-name"
        elif test_store == "staging":
            self.useFixture(StagingStore())
            self.register_count_limit = 100
            self.reserved_snap_name = "snapcraft-reserved-snap-name"
        elif test_store == "production":
            # Use the default server URLs
            self.register_count_limit = 10
            self.reserved_snap_name = "bash"
        else:
            raise ValueError("Unknown test store option: {}".format(test_store))

        # Do not change this email address. If you use a different address, it
        # will pollute KPIs, so always notify the store team if you need to use
        # a different value.
        self.user_email = (
            os.getenv("TEST_USER_EMAIL") or "snapcraft-test+user@canonical.com"
        )
        self.test_track_snap = os.getenv(
            "TEST_SNAP_WITH_TRACKS", "test-snapcraft-tracks"
        )
        if test_store == "fake":
            self.user_password = "test correct password"
        else:
            self.user_password = os.getenv("TEST_USER_PASSWORD")


class GitRepo(fixtures.Fixture):
    """Create a git repo in the current directory"""

    def setUp(self):
        super().setUp()
        name = "git-source"  # must match what the tests expect

        def _add_and_commit_file(path, filename, contents=None, message=None):
            if not contents:
                contents = filename
            if not message:
                message = filename

            with open(os.path.join(path, filename), "w") as fp:
                fp.write(contents)

            call(["git", "-C", name, "add", filename])
            call(["git", "-C", name, "commit", "-am", message])

        os.makedirs(name)
        call(["git", "-C", name, "init"])
        call(["git", "-C", name, "config", "user.name", "Test User"])
        call(["git", "-C", name, "config", "user.email", "testuser@example.com"])

        _add_and_commit_file(name, "testing")
        call(["git", "-C", name, "branch", "test-branch"])

        _add_and_commit_file(name, "testing-2")
        call(["git", "-C", name, "tag", "feature-tag"])

        _add_and_commit_file(name, "testing-3")

        self.commit = call_with_output(["git", "-C", name, "rev-parse", "HEAD"])


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

        bzr_home = self.useFixture(fixtures.TempDir()).path
        self.useFixture(fixtures.EnvironmentVariable("BZR_HOME", bzr_home))
        self.useFixture(
            fixtures.EnvironmentVariable(
                "BZR_EMAIL", "Test User <test.user@example.com>"
            )
        )

        with return_to_cwd():
            os.makedirs(self.name)
            os.chdir(self.name)
            call(["bzr", "init"])
            with open("testing", "w") as fp:
                fp.write("testing")

            call(["bzr", "add", "testing"])
            call(["bzr", "commit", "-m", "testing"])
            call(["bzr", "tag", "feature-tag"])
            revno = call_with_output(["bzr", "revno"])

            self.commit = revno


class SvnRepo(fixtures.Fixture):
    def __init__(self, name):
        self.name = name

    def setUp(self):
        super().setUp()

        working_tree = "svn-repo"
        call(["svnadmin", "create", self.name])
        call(
            [
                "svn",
                "checkout",
                "file://{}".format(os.path.join(os.getcwd(), self.name)),
                working_tree,
            ]
        )

        with return_to_cwd():
            os.chdir(working_tree)
            with open("testing", "w") as fp:
                fp.write("testing")

            call(["svn", "add", "testing"])
            call(["svn", "commit", "-m", "svn testing"])
            revno = "1"

            self.commit = revno


class HgRepo(fixtures.Fixture):
    def __init__(self, name):
        self.name = name

    def setUp(self):
        super().setUp()

        with return_to_cwd():
            os.makedirs(self.name)
            os.chdir(self.name)
            call(["hg", "init"])
            with open("testing", "w") as fp:
                fp.write("testing")

            call(["hg", "add", "testing"])
            call(
                [
                    "hg",
                    "commit",
                    "-m",
                    "testing",
                    "-u",
                    "Test User <test.user@example.com>",
                ]
            )
            call(
                ["hg", "tag", "feature-tag", "-u", "Test User <test.user@example.com>"]
            )
            revno = call_with_output(["hg", "id"]).split()[0]

            self.commit = revno


class WithoutSnapInstalled(fixtures.Fixture):
    """Assert that a snap is not installed and remove it on clean up.

    :raises: AssertionError: if the snap is installed when this fixture is
        set up.
    """

    def __init__(self, snap_name: str) -> None:
        super().__init__()
        self.snap_name = snap_name.split("/")[0]

    def setUp(self) -> None:
        super().setUp()
        with contextlib.suppress(subprocess.CalledProcessError):
            subprocess.check_call(["snap", "list", self.snap_name])
            raise AssertionError(
                "This test cannot run if you already have the {snap!r} snap "
                "installed. Please uninstall it by running "
                "'sudo snap remove {snap}'.".format(snap=self.snap_name)
            )

        self.addCleanup(self._remove_snap)

    def _remove_snap(self) -> None:
        try:
            subprocess.check_output(
                ["sudo", "snap", "remove", self.snap_name], stderr=subprocess.STDOUT
            )
        except subprocess.CalledProcessError as e:
            RuntimeError("unable to remove {!r}: {}".format(self.snap_name, e.output))


class SnapcraftYaml(fixtures.Fixture):
    def __init__(  # noqa: C901
        self,
        path,
        name="test-snap",
        base="core18",
        build_base=None,
        version="test-version",
        summary="test-summary",
        description="test-description",
        confinement="strict",
        architectures=None,
        apps=None,
        hooks=None,
        parts=None,
        type="app",
    ):
        super().__init__()

        if apps is None:
            apps = dict()

        if parts is None:
            parts = dict()

        self.path = path
        self.data = {
            "confinement": confinement,
            "parts": parts,
            "apps": apps,
            "type": type,
        }
        if name is not None:
            self.data["name"] = name
        if version is not None:
            self.data["version"] = version
        if summary is not None:
            self.data["summary"] = summary
        if description is not None:
            self.data["description"] = description
        if architectures is not None:
            self.data["architectures"] = architectures
        if base is not None:
            self.data["base"] = base
        if build_base is not None:
            self.data["build-base"] = build_base
        if hooks is not None:
            self.data["hooks"] = hooks

    def update_part(self, name, data):
        part = {name: data}
        self.data["parts"].update(part)
        self.write_snapcraft_yaml()

    def update_app(self, name, data):
        app = {name: data}
        self.data["apps"].update(app)
        self.write_snapcraft_yaml()

    def write_snapcraft_yaml(self):
        self.snapcraft_yaml_file_path = os.path.join(
            self.path, "snap", "snapcraft.yaml"
        )
        os.makedirs(os.path.join(self.path, "snap"), exist_ok=True)
        with open(self.snapcraft_yaml_file_path, "w") as snapcraft_yaml_file:
            yaml_utils.dump(self.data, stream=snapcraft_yaml_file)

    def _setUp(self):
        super()._setUp()
        self.write_snapcraft_yaml()


class SharedCache(fixtures.Fixture):
    def __init__(self, name) -> None:
        super().__init__()
        self.name = name

    def setUp(self) -> None:
        super().setUp()
        shared_cache_dir = os.path.join(
            tempfile.gettempdir(), "snapcraft_test_cache_{}".format(self.name)
        )
        os.makedirs(shared_cache_dir, exist_ok=True)
        self.useFixture(
            fixtures.EnvironmentVariable("XDG_CACHE_HOME", shared_cache_dir)
        )
        patcher = mock.patch(
            "xdg.BaseDirectory.xdg_cache_home", new=os.path.join(shared_cache_dir)
        )
        patcher.start()
        self.addCleanup(patcher.stop)


class FakeBaseEnvironment(fixtures.Fixture):

    _LINKER_FOR_ARCH = dict(
        armv7l="lib/ld-linux-armhf.so.3",
        aarch64="lib/ld-linux-aarch64.so.1",
        i686="lib/ld-linux.so.2",
        ppc64le="lib64/ld64.so.2",
        ppc="lib/ld-linux.so.2",
        x86_64="lib64/ld-linux-x86-64.so.2",
        s390x="lib/ld64.so.1",
    )

    _32BIT_USERSPACE_ARCHITECTURE = dict(
        aarch64="armv7l",
        armv8l="armv7l",
        ppc64le="ppc",
        x86_64="i686",
        i686="i686",
        armv7l="armv7l",
    )

    _WINDOWS_TRANSLATIONS = dict(AMD64="x86_64")

    def _get_platform_architecture(self):
        architecture = platform.machine()

        # Translate the windows architectures we know of to architectures
        # we can work with.
        if sys.platform == "win32":
            architecture = self._WINDOWS_TRANSLATIONS[architecture]

        if platform.architecture()[0] == "32bit":
            userspace = self._32BIT_USERSPACE_ARCHITECTURE[architecture]
            if userspace:
                architecture = userspace

        return architecture

    def __init__(self, *, machine=None):
        super().__init__()
        if machine is None:
            self._machine = self._get_platform_architecture()
        else:
            self._machine = machine

    def _setUp(self):
        super()._setUp()

        patcher = mock.patch("platform.machine")
        self.mock_machine = patcher.start()
        self.mock_machine.return_value = self._machine
        self.addCleanup(patcher.stop)

        self.core_path = self.useFixture(fixtures.TempDir()).path
        patcher = mock.patch("snapcraft.internal.common.get_installed_snap_path")
        mock_core_path = patcher.start()
        mock_core_path.return_value = self.core_path
        self.addCleanup(patcher.stop)

        self.content_dirs = set([])
        mock_content_dirs = fixtures.MockPatch(
            "snapcraft.project._project.Project._get_provider_content_dirs",
            return_value=self.content_dirs,
        )
        self.useFixture(mock_content_dirs)

        # Create file to represent the linker so it is found
        linker_path = os.path.join(self.core_path, self._LINKER_FOR_ARCH[self._machine])
        os.makedirs(os.path.dirname(linker_path), exist_ok=True)
        real_linker = os.path.join(self.core_path, "usr", "lib", "ld-2.23.so")
        os.makedirs(os.path.dirname(real_linker), exist_ok=True)
        open(real_linker, "w").close()
        os.symlink(
            os.path.relpath(real_linker, os.path.dirname(linker_path)), linker_path
        )


class FakeSnapcraftIsASnap(fixtures.Fixture):
    def _setUp(self):
        super()._setUp()

        self.useFixture(fixtures.EnvironmentVariable("SNAP", "/snap/snapcraft/current"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP_NAME", "snapcraft"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP_VERSION", "4.0"))
