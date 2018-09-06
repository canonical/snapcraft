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

import collections
import contextlib
import copy
import io
import os
import platform
import pkgutil
import shutil
import string
import subprocess
import sys
import tempfile
import textwrap
import threading
import urllib.parse
from functools import partial
from types import ModuleType
from unittest import mock
from subprocess import CalledProcessError
from typing import Callable

import fixtures
import xdg
import yaml

import snapcraft
from snapcraft.internal import elf
from tests import fake_servers
from tests.fake_servers import api, search, upload
from tests.file_utils import get_snapcraft_path
from tests.subprocess_utils import call, call_with_output


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


class FakeProjectOptions(fixtures.Fixture):
    def __init__(self, **kwargs):
        self._kwargs = dict(
            arch_triplet=kwargs.pop("arch_triplet", "x86_64-gnu-linux"),
            parts_dir=kwargs.pop("parts_dir", "parts"),
            stage_dir=kwargs.pop("stage_dir", "stage"),
            prime_dir=kwargs.pop("prime_dir", "prime"),
            parallel_build_count=kwargs.pop("parallel_build_count", "1"),
        )
        if kwargs:
            raise NotImplementedError(
                "Handling of {!r} is not implemented".format(kwargs.keys())
            )

    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.project.Project")
        patcher.start()
        self.addCleanup(patcher.stop)

        # Special handling is required as ProjectOptions attributes are
        # handled with the @property decorator.
        project_options_t = type(snapcraft.project.Project.return_value)
        for key in self._kwargs:
            setattr(project_options_t, key, self._kwargs[key])


class SilentSnapProgress(fixtures.Fixture):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.internal.lifecycle._packer.ProgressBar")
        patcher.start()
        self.addCleanup(patcher.stop)


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
                "UBUNTU_SSO_API_ROOT_URL",
                urllib.parse.urljoin(self.fake_sso_server_fixture.url, "api/v2/"),
            )
        )

        self.useFixture(fixtures.EnvironmentVariable("STORE_RETRIES", "1"))
        self.useFixture(fixtures.EnvironmentVariable("STORE_BACKOFF", "0"))

        self.fake_store_upload_server_fixture = FakeStoreUploadServerRunning()
        self.useFixture(self.fake_store_upload_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_STORE_UPLOAD_ROOT_URL",
                self.fake_store_upload_server_fixture.url,
            )
        )

        self.fake_store_api_server_fixture = FakeStoreAPIServerRunning(self)
        self.useFixture(self.fake_store_api_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_STORE_API_ROOT_URL",
                urllib.parse.urljoin(
                    self.fake_store_api_server_fixture.url, "dev/api/"
                ),
            )
        )

        self.fake_store_search_server_fixture = FakeStoreSearchServerRunning()
        self.useFixture(self.fake_store_search_server_fixture)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_STORE_SEARCH_ROOT_URL",
                self.fake_store_search_server_fixture.url,
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
                "UBUNTU_STORE_API_ROOT_URL",
                "https://dashboard.staging.snapcraft.io/dev/api/",
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_STORE_UPLOAD_ROOT_URL",
                "https://upload.apps.staging.ubuntu.com/",
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_SSO_API_ROOT_URL", "https://login.staging.ubuntu.com/api/v2/"
            )
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_STORE_SEARCH_ROOT_URL", "https://api.staging.snapcraft.io/"
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
            self.reserved_snap_name = "bash"
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


class FakePlugin(fixtures.Fixture):
    """Dynamically generate a new module containing the provided plugin"""

    def __init__(self, plugin_name, plugin_class):
        super().__init__()
        self._import_name = "snapcraft.plugins.{}".format(plugin_name.replace("-", "_"))
        self._plugin_class = plugin_class

    def _setUp(self):
        plugin_module = ModuleType(self._import_name)
        setattr(plugin_module, self._plugin_class.__name__, self._plugin_class)
        sys.modules[self._import_name] = plugin_module
        self.addCleanup(self._remove_module)

    def _remove_module(self):
        del sys.modules[self._import_name]


class FakeMetadataExtractor(fixtures.Fixture):
    """Dynamically generate a new module containing the provided extractor"""

    def __init__(
        self,
        extractor_name: str,
        extractor: Callable[[str], snapcraft.extractors.ExtractedMetadata],
        exported_name="extract",
    ) -> None:
        super().__init__()
        self._extractor_name = extractor_name
        self._exported_name = exported_name
        self._import_name = "snapcraft.extractors.{}".format(extractor_name)
        self._extractor = extractor

    def _setUp(self) -> None:
        extractor_module = ModuleType(self._import_name)
        setattr(extractor_module, self._exported_name, self._extractor)
        sys.modules[self._import_name] = extractor_module
        self.addCleanup(self._remove_module)

        real_iter_modules = pkgutil.iter_modules

        def _fake_iter_modules(path):
            if path == snapcraft.extractors.__path__:
                yield None, self._extractor_name, False
            else:
                yield real_iter_modules(path)

        patcher = mock.patch("pkgutil.iter_modules", new=_fake_iter_modules)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _remove_module(self) -> None:
        del sys.modules[self._import_name]


class FakeLXD(fixtures.Fixture):
    """..."""

    def __init__(self):
        self.status = None
        self.files = []
        self.kernel_arch = "x86_64"
        self.devices = "{}"

    def _setUp(self):
        patcher = mock.patch("subprocess.check_call")
        self.check_call_mock = patcher.start()
        self.check_call_mock.side_effect = self.check_output_side_effect()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.check_output_mock.side_effect = self.check_output_side_effect()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("time.sleep", lambda _: None)
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("platform.machine")
        self.machine_mock = patcher.start()
        self.machine_mock.return_value = "x86_64"
        self.addCleanup(patcher.stop)
        patcher = mock.patch("platform.architecture")
        self.architecture_mock = patcher.start()
        self.architecture_mock.return_value = ("64bit", "ELF")
        self.addCleanup(patcher.stop)

    def call_effect(self, *args, **kwargs):
        if args[0] == ["lxc", "remote", "get-default"]:
            return "local".encode("utf-8")
        elif args[0][:2] == ["lxc", "info"]:
            return "Architecture: {}".format(self.kernel_arch).encode("utf-8")
        elif args[0][:3] == ["lxc", "list", "--format=json"]:
            if self.status and args[0][3] == self.name:
                return (
                    string.Template(
                        """
                    [{"name": "$NAME",
                      "status": "$STATUS",
                      "devices": $DEVICES}]
                    """
                    )
                    .substitute(
                        {
                            # Container name without remote prefix
                            "NAME": self.name.split(":")[-1],
                            "STATUS": self.status,
                            "DEVICES": self.devices,
                        }
                    )
                    .encode("utf-8")
                )
            return "[]".encode("utf-8")
        elif args[0][0] == "lxc" and args[0][1] in ["init", "start", "launch", "stop"]:
            return self._lxc_create_start_stop(args)
        elif args[0][:2] == ["lxc", "exec"]:
            return self._lxc_exec(args)
        elif args[0][:4] == ["lxc", "image", "list", "--format=json"]:
            return (
                '[{"architecture":"test-architecture",'
                '"fingerprint":"test-fingerprint",'
                '"created_at":"test-created-at"}]'
            ).encode("utf-8")
        elif args[0][0] == "sha384sum":
            return "deadbeef {}".format(args[0][1]).encode("utf-8")
        else:
            return "".encode("utf-8")

    def check_output_side_effect(self):
        return self.call_effect

    def _lxc_create_start_stop(self, args):
        if args[0][1] == "init":
            self.name = args[0][3]
            self.status = "Stopped"
        elif args[0][1] == "launch":
            self.name = args[0][4]
            self.status = "Running"
        elif args[0][1] == "start" and self.name == args[0][2]:
            self.status = "Running"
        elif args[0][1] == "stop" and not self.status:
            # error: not found
            raise CalledProcessError(returncode=1, cmd=args[0])

    def _lxc_exec(self, args):
        if self.status and args[0][2] == self.name:
            cmd = args[0][4]
            if cmd == "sudo":
                cmd = args[0][8]
            if cmd == "ls":
                return " ".join(self.files).encode("utf-8")
            elif cmd == "readlink":
                if args[0][-1].endswith("/current"):
                    raise CalledProcessError(returncode=1, cmd=cmd)
            elif "sha384sum" in args[0][-1]:
                raise CalledProcessError(returncode=1, cmd=cmd)

    def _popen(self, args):
        class Popen:
            def __init__(self, args):
                self.args = args

            def terminate(self):
                pass

        return Popen(args)


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


class FakeAptCache(fixtures.Fixture):
    class Cache:
        def __init__(self):
            super().__init__()
            self.packages = collections.OrderedDict()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def __setitem__(self, key, item):
            package_parts = key.split("=")
            package_name = package_parts[0]
            version = package_parts[1] if len(package_parts) > 1 else item.version
            if package_name in self.packages:
                self.packages[package_name].version = version
            else:
                if version and not item.version:
                    item.version = version
                self.packages[package_name] = item

        def __getitem__(self, key):
            if "=" in key:
                key = key.split("=")[0]
            return self.packages[key]

        def __contains__(self, key):
            return key in self.packages

        def __iter__(self):
            return iter(self.packages.values())

        def open(self):
            pass

        def close(self):
            pass

        def update(self, *args, **kwargs):
            pass

        def get_changes(self):
            return [
                self.packages[package]
                for package in self.packages
                if self.packages[package].marked_install
            ]

        def get_providing_packages(self, package_name):
            providing_packages = []
            for package in self.packages:
                if package_name in self.packages[package].provides:
                    providing_packages.append(self.packages[package])
            return providing_packages

        def is_virtual_package(self, package_name):
            is_virtual = False
            if package_name not in self.packages:
                for package in self.packages:
                    if package_name in self.packages[package].provides:
                        return True
            return is_virtual

    def __init__(self, packages=None):
        super().__init__()
        self.packages = packages if packages else []

    def setUp(self):
        super().setUp()
        temp_dir_fixture = fixtures.TempDir()
        self.useFixture(temp_dir_fixture)
        self.path = temp_dir_fixture.path
        patcher = mock.patch("snapcraft.repo._deb.apt.Cache")
        self.mock_apt_cache = patcher.start()
        self.addCleanup(patcher.stop)

        self.cache = self.Cache()
        self.mock_apt_cache.return_value = self.cache
        for package, version in self.packages:
            self.add_package(FakeAptCachePackage(package, version))

        def fetch_binary(package_candidate, destination):
            path = os.path.join(self.path, "{}.deb".format(package_candidate.name))
            open(path, "w").close()
            return path

        patcher = mock.patch("snapcraft.repo._deb._AptCache.fetch_binary")
        mock_fetch_binary = patcher.start()
        mock_fetch_binary.side_effect = fetch_binary
        self.addCleanup(patcher.stop)

        # Add all the packages in the manifest.
        with open(
            os.path.join(
                get_snapcraft_path(), "snapcraft", "internal", "repo", "manifest.txt"
            )
        ) as manifest_file:
            self.add_packages([line.strip() for line in manifest_file])

    def add_package(self, package):
        package.temp_dir = self.path
        self.cache[package.name] = package

    def add_packages(self, package_names):
        for name in package_names:
            self.cache[name] = FakeAptCachePackage(name)


class FakeAptCachePackage:
    def __init__(
        self,
        name,
        version=None,
        installed=None,
        temp_dir=None,
        provides=None,
        priority="non-essential",
    ):
        super().__init__()
        self.temp_dir = temp_dir
        self.name = name
        self._version = None
        self.versions = {}
        self.version = version
        self.candidate = self
        self.dependencies = []
        self.conflicts = []
        self.provides = provides if provides else []
        if installed:
            # XXX The installed attribute requires some values that the fake
            # package also requires. The shortest path to do it that I found
            # was to get installed to return the same fake package.
            self.installed = self
        else:
            self.installed = None
        self.priority = priority
        self.marked_install = False
        self.is_auto_installed = False

    def __str__(self):
        if "=" in self.name:
            return self.name
        else:
            return "{}={}".format(self.name, self.version)

    @property
    def version(self):
        return self._version

    @property
    def is_auto_removable(self):
        return self.marked_install and self.is_auto_installed

    @version.setter
    def version(self, version):
        self._version = version
        if version is not None:
            self.versions.update({version: self})

    def mark_install(self, *, auto_fix=True, from_user=True):
        if not self.installed:
            # First, verify dependencies are valid. If not, bail.
            for or_set in self.dependencies:
                for dep in or_set:
                    if "broken" in dep.name:
                        return

            for or_set in self.dependencies:
                if or_set and or_set[0].target_versions:
                    # Install the first target version of the first OR
                    or_set[0].target_versions[0].mark_install(
                        auto_fix=auto_fix, from_user=from_user
                    )
            for conflict in self.conflicts:
                conflict.mark_keep()

            self.marked_install = True
            self.is_auto_installed = not from_user

    def mark_auto(self, auto=True):
        self.is_auto_installed = auto

    def mark_keep(self):
        self.marked_install = False
        self.is_auto_installed = False

    def get_dependencies(self, _):
        return []


class FakeAptBaseDependency:
    def __init__(self, name, target_versions):
        self.name = name
        self.target_versions = target_versions


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
        if snapcraft.repo.snaps.SnapPackage.is_snap_installed(self.snap_name):
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
    def __init__(
        self,
        path,
        name="test-snap",
        version="test-version",
        summary="test-summary",
        description="test-description",
        confinement="strict",
        architectures=None,
    ):
        super().__init__()
        self.path = path
        self.data = {"confinement": confinement, "parts": {}, "apps": {}}
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

    def update_part(self, name, data):
        part = {name: data}
        self.data["parts"].update(part)

    def update_app(self, name, data):
        app = {name: data}
        self.data["apps"].update(app)

    def _setUp(self):
        super()._setUp()
        self.snapcraft_yaml_file_path = os.path.join(
            self.path, "snap", "snapcraft.yaml"
        )
        os.makedirs(os.path.join(self.path, "snap"), exist_ok=True)
        with open(self.snapcraft_yaml_file_path, "w") as snapcraft_yaml_file:
            yaml.dump(self.data, snapcraft_yaml_file)


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


def _fake_elffile_extract(self, path):
    arch = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
    name = os.path.basename(path)
    if name in [
        "fake_elf-2.26",
        "fake_elf-bad-ldd",
        "fake_elf-with-core-libs",
        "fake_elf-bad-patchelf",
    ]:
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.26")
        return (arch, "/lib64/ld-linux-x86-64.so.2", "", {glibc.name: glibc}, False)
    elif name == "fake_elf-2.23":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.23")
        return (arch, "/lib64/ld-linux-x86-64.so.2", "", {glibc.name: glibc}, False)
    elif name == "fake_elf-1.1":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_1.1")
        glibc.add_version("GLIBC_0.1")
        return (arch, "/lib64/ld-linux-x86-64.so.2", "", {glibc.name: glibc}, False)
    elif name == "fake_elf-static":
        return arch, "", "", {}, False
    elif name == "fake_elf-shared-object":
        openssl = elf.NeededLibrary(name="libssl.so.1.0.0")
        openssl.add_version("OPENSSL_1.0.0")
        return arch, "", "libfake_elf.so.0", {openssl.name: openssl}, False
    elif name == "fake_elf-with-execstack":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")
        return (arch, "/lib64/ld-linux-x86-64.so.2", "", {glibc.name: glibc}, True)
    elif name == "fake_elf-with-bad-execstack":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")
        return (arch, "/lib64/ld-linux-x86-64.so.2", "", {glibc.name: glibc}, True)
    elif name == "libc.so.6":
        return arch, "", "libc.so.6", {}, False
    elif name == "libssl.so.1.0.0":
        return arch, "", "libssl.so.1.0.0", {}, False
    else:
        return arch, "", "", {}, False


class FakeElf(fixtures.Fixture):
    def __getitem__(self, item):
        return self._elf_files[item]

    def __init__(self, *, root_path, patchelf_version="0.10"):
        super().__init__()

        self.root_path = root_path
        self.core_base_path = None
        self._patchelf_version = patchelf_version

    def _setUp(self):
        super()._setUp()

        self.core_base_path = self.useFixture(fixtures.TempDir()).path

        binaries_path = os.path.join(get_snapcraft_path(), "tests", "bin", "elf")

        new_binaries_path = self.useFixture(fixtures.TempDir()).path
        current_path = os.environ.get("PATH")
        new_path = "{}:{}".format(new_binaries_path, current_path)
        self.useFixture(fixtures.EnvironmentVariable("PATH", new_path))

        # Copy strip
        for f in ["strip", "execstack"]:
            shutil.copy(
                os.path.join(binaries_path, f), os.path.join(new_binaries_path, f)
            )
            os.chmod(os.path.join(new_binaries_path, f), 0o755)

        # Some values in ldd need to be set with core_path
        with open(os.path.join(binaries_path, "ldd")) as rf:
            with open(os.path.join(new_binaries_path, "ldd"), "w") as wf:
                for line in rf.readlines():
                    wf.write(line.replace("{CORE_PATH}", self.core_base_path))
        os.chmod(os.path.join(new_binaries_path, "ldd"), 0o755)

        # Some values in ldd need to be set with core_path
        self.patchelf_path = os.path.join(new_binaries_path, "patchelf")
        with open(os.path.join(binaries_path, "patchelf")) as rf:
            with open(self.patchelf_path, "w") as wf:
                for line in rf.readlines():
                    wf.write(line.replace("{VERSION}", self._patchelf_version))
        os.chmod(os.path.join(new_binaries_path, "patchelf"), 0o755)

        patcher = mock.patch.object(
            elf.ElfFile, "_extract", new_callable=lambda: _fake_elffile_extract
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        self._elf_files = {
            "fake_elf-2.26": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-2.26")
            ),
            "fake_elf-2.23": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-2.23")
            ),
            "fake_elf-1.1": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-1.1")
            ),
            "fake_elf-static": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-static")
            ),
            "fake_elf-shared-object": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-shared-object")
            ),
            "fake_elf-bad-ldd": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-bad-ldd")
            ),
            "fake_elf-bad-patchelf": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-bad-patchelf")
            ),
            "fake_elf-with-core-libs": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-core-libs")
            ),
            "fake_elf-with-execstack": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-execstack")
            ),
            "fake_elf-with-bad-execstack": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-bad-execstack")
            ),
            "libc.so.6": elf.ElfFile(path=os.path.join(self.root_path, "libc.so.6")),
            "libssl.so.1.0.0": elf.ElfFile(
                path=os.path.join(self.root_path, "libssl.so.1.0.0")
            ),
        }

        for elf_file in self._elf_files.values():
            with open(elf_file.path, "wb") as f:
                f.write(b"\x7fELF")
                if elf_file.path.endswith("fake_elf-bad-patchelf"):
                    f.write(b"nointerpreter")

        self.root_libraries = {"foo.so.1": os.path.join(self.root_path, "foo.so.1")}

        for root_library in self.root_libraries.values():
            with open(root_library, "wb") as f:
                f.write(b"\x7fELF")


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
        patcher = mock.patch("snapcraft.internal.common.get_core_path")
        mock_core_path = patcher.start()
        mock_core_path.return_value = self.core_path
        self.addCleanup(patcher.stop)

        # Create file to represent the linker so it is found
        linker_path = os.path.join(self.core_path, self._LINKER_FOR_ARCH[self._machine])
        os.makedirs(os.path.dirname(linker_path), exist_ok=True)
        real_linker = os.path.join(self.core_path, "usr", "lib", "ld-2.23.so")
        os.makedirs(os.path.dirname(real_linker), exist_ok=True)
        open(real_linker, "w").close()
        os.symlink(
            os.path.relpath(real_linker, os.path.dirname(linker_path)), linker_path
        )


class FakeSnapcraftctl(fixtures.Fixture):
    def _setUp(self):
        super()._setUp()

        snapcraft_path = get_snapcraft_path()

        tempdir = self.useFixture(fixtures.TempDir()).path
        altered_path = "{}:{}".format(tempdir, os.environ.get("PATH"))
        self.useFixture(fixtures.EnvironmentVariable("PATH", altered_path))

        snapcraftctl_path = os.path.join(tempdir, "snapcraftctl")
        with open(snapcraftctl_path, "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                #!/usr/bin/env python3

                # Make sure we can find snapcraft, even if it's not installed
                # (like in CI).
                import sys
                sys.path.append('{snapcraft_path!s}')

                import snapcraft.cli.__main__

                if __name__ == '__main__':
                    snapcraft.cli.__main__.run_snapcraftctl(
                        prog_name='snapcraftctl')
            """.format(
                        snapcraft_path=snapcraft_path
                    )
                )
            )
            f.flush()

        os.chmod(snapcraftctl_path, 0o755)


class FakeSnapcraftIsASnap(fixtures.Fixture):
    def _setUp(self):
        super()._setUp()

        self.useFixture(fixtures.EnvironmentVariable("SNAP", "/snap/snapcraft/current"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP_NAME", "snapcraft"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP_VERSION", "devel"))


class FakeExtension(fixtures.Fixture):
    """Dynamically generate a new module containing the provided extension"""

    def __init__(self, extension_name, extension_class):
        super().__init__()
        self._import_name = "snapcraft.internal.project_loader._extensions.{}".format(
            extension_name
        )
        self._extension_class = extension_class

    def _setUp(self):
        extension_module = ModuleType(self._import_name)
        setattr(extension_module, self._extension_class.__name__, self._extension_class)
        sys.modules[self._import_name] = extension_module
        self.addCleanup(self._remove_module)

    def _remove_module(self):
        del sys.modules[self._import_name]
