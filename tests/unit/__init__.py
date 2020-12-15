# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import http.server
import logging
import os
import stat
import threading
from unittest import mock

import apt
import fixtures
import progressbar
import testscenarios
import testtools

from snapcraft.internal import common, steps
from tests import fake_servers, fixture_setup
from tests.file_utils import get_snapcraft_path
from tests.unit.part_loader import load_part


class ContainsList(list):
    def __eq__(self, other):
        return all([i[0] in i[1] for i in zip(self, other)])


class MockOptions:
    def __init__(
        self,
        source=None,
        source_type=None,
        source_branch=None,
        source_tag=None,
        source_subdir=None,
        source_depth=None,
        source_commit=None,
        source_checksum=None,
        disable_parallel=False,
    ):
        self.source = source
        self.source_type = source_type
        self.source_depth = source_depth
        self.source_branch = source_branch
        self.source_commit = source_commit
        self.source_tag = source_tag
        self.source_subdir = source_subdir
        self.disable_parallel = disable_parallel


class IsExecutable:
    """Match if a file path is executable."""

    def __str__(self):
        return "IsExecutable()"

    def match(self, file_path):
        if not os.stat(file_path).st_mode & stat.S_IEXEC:
            return testtools.matchers.Mismatch(
                "Expected {!r} to be executable, but it was not".format(file_path)
            )
        return None


class LinkExists:
    """Match if a file path is a symlink."""

    def __init__(self, expected_target=None):
        self._expected_target = expected_target

    def __str__(self):
        return "LinkExists()"

    def match(self, file_path):
        if not os.path.exists(file_path):
            return testtools.matchers.Mismatch(
                "Expected {!r} to be a symlink, but it doesn't exist".format(file_path)
            )

        if not os.path.islink(file_path):
            return testtools.matchers.Mismatch(
                "Expected {!r} to be a symlink, but it was not".format(file_path)
            )

        target = os.readlink(file_path)
        if target != self._expected_target:
            return testtools.matchers.Mismatch(
                "Expected {!r} to be a symlink pointing to {!r}, but it was "
                "pointing to {!r}".format(file_path, self._expected_target, target)
            )

        return None


class TestCase(testscenarios.WithScenarios, testtools.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fake_snapd = fixture_setup.FakeSnapd()
        cls.fake_snapd.setUp()

    @classmethod
    def tearDownClass(cls):
        cls.fake_snapd.cleanUp()

    def setUp(self):
        super().setUp()
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        # Use a separate path for XDG dirs, or changes there may be detected as
        # source changes.
        self.xdg_path = self.useFixture(fixtures.TempDir()).path
        self.useFixture(fixture_setup.TempXDG(self.xdg_path))
        self.fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)
        # Some tests will directly or indirectly change the plugindir, which
        # is a module variable. Make sure that it is returned to the original
        # value when a test ends.
        self.addCleanup(common.set_plugindir, common.get_plugindir())
        self.addCleanup(common.set_schemadir, common.get_schemadir())
        self.addCleanup(common.set_extensionsdir, common.get_extensionsdir())
        self.addCleanup(common.set_keyringsdir, common.get_keyringsdir())
        self.addCleanup(common.reset_env)
        common.set_schemadir(os.path.join(get_snapcraft_path(), "schema"))
        self.fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(self.fake_logger)

        # Some tests will change the apt Dir::Etc::Trusted and
        # Dir::Etc::TrustedParts directories. Make sure they're properly reset.
        self.addCleanup(
            apt.apt_pkg.config.set,
            "Dir::Etc::Trusted",
            apt.apt_pkg.config.find_file("Dir::Etc::Trusted"),
        )
        self.addCleanup(
            apt.apt_pkg.config.set,
            "Dir::Etc::TrustedParts",
            apt.apt_pkg.config.find_file("Dir::Etc::TrustedParts"),
        )

        patcher = mock.patch("os.sched_getaffinity")
        self.cpu_count = patcher.start()
        self.cpu_count.return_value = {1, 2}
        self.addCleanup(patcher.stop)

        # We do not want the paths to affect every test we have.
        patcher = mock.patch(
            "snapcraft.file_utils.get_snap_tool_path", side_effect=lambda x: x
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.internal.indicators.ProgressBar", new=SilentProgressBar
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        # These are what we expect by default
        self.snap_dir = os.path.join(os.getcwd(), "snap")
        self.prime_dir = os.path.join(os.getcwd(), "prime")
        self.stage_dir = os.path.join(os.getcwd(), "stage")
        self.parts_dir = os.path.join(os.getcwd(), "parts")
        self.local_plugins_dir = os.path.join(self.snap_dir, "plugins")

        # Use this host to run through the lifecycle tests
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )

        # Make sure snap installation does the right thing
        self.fake_snapd.installed_snaps = [
            dict(name="core16", channel="stable", revision="10"),
            dict(name="core18", channel="stable", revision="10"),
        ]
        self.fake_snapd.snaps_result = [
            dict(name="core16", channel="stable", revision="10"),
            dict(name="core18", channel="stable", revision="10"),
        ]
        self.fake_snapd.find_result = [
            dict(
                core16=dict(
                    channel="stable",
                    channels={"latest/stable": dict(confinement="strict")},
                )
            ),
            dict(
                core18=dict(
                    channel="stable",
                    channels={"latest/stable": dict(confinement="strict")},
                )
            ),
        ]
        self.fake_snapd.snap_details_func = None

        self.fake_snap_command = fixture_setup.FakeSnapCommand()
        self.useFixture(self.fake_snap_command)

        # Avoid installing patchelf in the tests
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_NO_PATCHELF", "1"))

        # Disable Sentry reporting for tests, otherwise they'll hang waiting
        # for input
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_ERROR_REPORTING", "false")
        )

        # Don't let the managed host variable leak into tests
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_MANAGED_HOST"))

        machine = os.environ.get("SNAPCRAFT_TEST_MOCK_MACHINE", None)
        self.base_environment = fixture_setup.FakeBaseEnvironment(machine=machine)
        self.useFixture(self.base_environment)

        # Make sure "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" is reset between tests
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG")
        )
        self.useFixture(fixture_setup.FakeSnapcraftctl())

        # Don't let host SNAPCRAFT_BUILD_INFO variable leak into tests
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_INFO"))

    def make_snapcraft_yaml(self, content, encoding="utf-8", location=""):
        snap_dir = os.path.join(location, "snap")
        os.makedirs(snap_dir, exist_ok=True)
        snapcraft_yaml = os.path.join(snap_dir, "snapcraft.yaml")
        with open(snapcraft_yaml, "w", encoding=encoding) as fp:
            fp.write(content)
        return snapcraft_yaml

    def verify_state(self, part_name, state_dir, expected_step_name):
        self.assertTrue(
            os.path.isdir(state_dir),
            "Expected state directory for {}".format(part_name),
        )

        # Expect every step up to and including the specified one to be run
        step = steps.get_step_by_name(expected_step_name)
        for step in step.previous_steps() + [step]:
            self.assertTrue(
                os.path.exists(os.path.join(state_dir, step.name)),
                "Expected {!r} to be run for {}".format(step.name, part_name),
            )

    def load_part(
        self,
        part_name,
        plugin_name=None,
        part_properties=None,
        project=None,
        stage_packages_repo=None,
        snap_name="test-snap",
        base="core18",
        build_base=None,
        confinement="strict",
        snap_type="app",
    ):
        return load_part(
            part_name=part_name,
            plugin_name=plugin_name,
            part_properties=part_properties,
            project=project,
            stage_packages_repo=stage_packages_repo,
            snap_name=snap_name,
            base=base,
            build_base=build_base,
            confinement=confinement,
            snap_type=snap_type,
        )


class TestWithFakeRemoteParts(TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeParts())


class FakeFileHTTPServerBasedTestCase(TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixtures.EnvironmentVariable("no_proxy", "localhost,127.0.0.1"))
        self.server = http.server.HTTPServer(
            ("127.0.0.1", 0), fake_servers.FakeFileHTTPRequestHandler
        )
        server_thread = threading.Thread(target=self.server.serve_forever)
        self.addCleanup(server_thread.join)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        server_thread.start()


class SilentProgressBar(progressbar.ProgressBar):
    """A progress bar causing no spurious output during tests."""

    def start(self):
        pass

    def update(self, value=None):
        pass

    def finish(self):
        pass
