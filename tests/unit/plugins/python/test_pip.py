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

import os
import subprocess

import fixtures
from unittest import mock

from testtools.matchers import Contains, Equals, HasLength

from snapcraft.plugins._python import _pip, errors

from ._basesuite import PythonBaseTestCase


class PipRunBaseTestCase(PythonBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.internal.common.run_output")
        self.mock_run_output = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.common.run")
        self.mock_run = patcher.start()
        self.addCleanup(patcher.stop)

        # Return json indicating that wheel and setuptools are installed
        self.mock_run_output.return_value = (
            '[{"name": "wheel", "version": "1.0"},'
            '{"name": "setuptools", "version": "1.0"}]'
        )

    def _assert_expected_enviroment(self, expected_python, headers_path):
        _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        ).setup()

        class check_env:
            def __init__(self, test):
                self.test = test

            def __eq__(self, env):
                self.test.assertThat(env, Contains("PYTHONUSERBASE"))
                self.test.assertThat(env["PYTHONUSERBASE"], Equals("install_dir"))

                self.test.assertThat(env, Contains("PYTHONHOME"))
                if expected_python.startswith("install_dir"):
                    self.test.assertThat(
                        env["PYTHONHOME"], Equals(os.path.join("install_dir", "usr"))
                    )
                else:
                    self.test.assertThat(
                        env["PYTHONHOME"], Equals(os.path.join("stage_dir", "usr"))
                    )

                self.test.assertThat(env, Contains("PATH"))
                self.test.assertThat(
                    env["PATH"], Contains(os.path.join("install_dir", "usr", "bin"))
                )

                if headers_path:
                    self.test.assertThat(env, Contains("CPPFLAGS"))
                    self.test.assertThat(
                        env["CPPFLAGS"], Contains("-I{}".format(headers_path))
                    )

                return True

        self.mock_run_output.assert_has_calls(
            [
                mock.call(
                    [expected_python, "-m", "pip"],
                    stderr=subprocess.STDOUT,
                    env=check_env(self),
                ),
                mock.call(
                    [expected_python, "-m", "pip", "list", "--format=json"],
                    env=mock.ANY,
                ),
                mock.call(
                    [expected_python, "-m", "pip", "list", "--format=json"],
                    env=mock.ANY,
                ),
            ]
        )


class PipRunTestCase(PipRunBaseTestCase):
    def test_environment_part_python_without_headers(self):
        expected_python = self._create_python_binary("install_dir")
        self._assert_expected_enviroment(expected_python, None)

    def test_environment_part_python_with_staged_headers(self):
        expected_python = self._create_python_binary("install_dir")
        # First, create staged headers
        staged_headers = os.path.join("stage_dir", "usr", "include", "pythontest")
        os.makedirs(staged_headers)
        self._assert_expected_enviroment(expected_python, staged_headers)

    @mock.patch("glob.glob")
    def test_environment_part_python_with_host_headers(self, mock_glob):
        host_headers = os.path.join(os.sep, "usr", "include", "pythontest")

        # Fake out glob so it looks like the headers are installed on the host
        def _fake_glob(pattern):
            if pattern.startswith(os.sep):
                return [host_headers]
            return []

        mock_glob.side_effect = _fake_glob

        expected_python = self._create_python_binary("install_dir")
        self._assert_expected_enviroment(expected_python, host_headers)

    def test_environment_staged_python_without_headers(self):
        expected_python = self._create_python_binary("stage_dir")
        self._assert_expected_enviroment(expected_python, None)

    def test_environment_staged_python_with_staged_headers(self):
        # First, create staged headers
        staged_headers = os.path.join("stage_dir", "usr", "include", "pythontest")
        os.makedirs(staged_headers)

        # Also create staged python
        expected_python = self._create_python_binary("stage_dir")

        self._assert_expected_enviroment(expected_python, staged_headers)

    @mock.patch("glob.glob")
    def test_environment_staged_python_with_host_headers(self, mock_glob):
        host_headers = os.path.join(os.sep, "usr", "include", "pythontest")

        # Fake out glob so it looks like the headers are installed on the host
        def _fake_glob(pattern):
            if pattern.startswith(os.sep):
                return [host_headers]
            return []

        mock_glob.side_effect = _fake_glob

        expected_python = self._create_python_binary("stage_dir")
        self._assert_expected_enviroment(expected_python, host_headers)

    def test_with_extra_cppflags(self):
        """Verify that existing CPPFLAGS are preserved"""

        expected_python = self._create_python_binary("install_dir")

        self.useFixture(fixtures.EnvironmentVariable("CPPFLAGS", "-I/opt/include"))
        _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        ).setup()

        class check_env:
            def __init__(self, test):
                self.test = test

            def __eq__(self, env):
                self.test.assertThat(env, Contains("CPPFLAGS"))
                self.test.assertThat(env["CPPFLAGS"], Contains("-I/opt/include"))

                return True

        self.mock_run_output.assert_has_calls(
            [
                mock.call(
                    [expected_python, "-m", "pip"],
                    env=check_env(self),
                    stderr=subprocess.STDOUT,
                )
            ]
        )


class SetupTestCase(PipRunBaseTestCase):
    def setUp(self):
        super().setUp()

        self.command = [self._create_python_binary("install_dir"), "-m", "pip"]

    def _assert_check_for_pip(self):
        self.mock_run_output.assert_has_calls(
            [
                mock.call(self.command, stderr=subprocess.STDOUT, env=mock.ANY),
                mock.call(self.command + ["list", "--format=json"], env=mock.ANY),
                mock.call(self.command + ["list", "--format=json"], env=mock.ANY),
            ]
        )

    def test_setup_with_pip_installed(self):
        """Test that no attempt is made to reinstall pip"""

        # Since _run doesn't raise an exception indicating pip isn't installed,
        # it must be installed.

        # Verify that no attempt is made to reinstall pip
        _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        ).setup()

        self._assert_check_for_pip()
        self.mock_run.assert_not_called()

    def test_setup_without_pip_installed(self):
        """Test that the system pip is used to install our own pip"""

        # Raise an exception indicating that pip isn't installed
        def fake_run(command, **kwargs):
            if command == self.command:
                raise subprocess.CalledProcessError(1, "foo", b"no module named pip")

            return "[]"

        self.mock_run_output.side_effect = fake_run

        # Verify that pip is then installed
        _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        ).setup()

        part_pythonhome = os.path.join("install_dir", "usr")
        host_pythonhome = os.path.join(os.path.sep, "usr")

        # What we're asserting here:
        # 1. That we test for the installed pip
        # 2. That we then download pip (and associated tools) using host pip
        # 3. That we then install pip (and associated tools) using host pip

        # Check that we correctly test for the installed pip
        self.mock_run_output.assert_has_calls(
            [
                mock.call(
                    self.command,
                    stderr=subprocess.STDOUT,
                    env=_CheckPythonhomeEnv(self, part_pythonhome),
                ),
                mock.call(self.command + ["list", "--format=json"], env=mock.ANY),
                mock.call(self.command + ["list", "--format=json"], env=mock.ANY),
            ]
        )

        # Now test that we download and install pip using the host's pip, and
        # then install the associated tools using the pip we pulled.
        self.assertThat(self.mock_run.mock_calls, HasLength(6))
        self.mock_run.assert_has_calls(
            [
                mock.call(
                    _CheckCommand(self, "download", ["pip"], []),
                    env=_CheckPythonhomeEnv(self, host_pythonhome),
                    cwd=None,
                ),
                mock.call(
                    _CheckCommand(self, "install", ["pip"], ["--ignore-installed"]),
                    env=_CheckPythonhomeEnv(self, host_pythonhome),
                    cwd=None,
                ),
                mock.call(
                    _CheckCommand(self, "download", ["wheel"], []),
                    env=_CheckPythonhomeEnv(self, part_pythonhome),
                    cwd=None,
                ),
                mock.call(
                    _CheckCommand(self, "install", ["wheel"], ["--ignore-installed"]),
                    env=_CheckPythonhomeEnv(self, part_pythonhome),
                    cwd=None,
                ),
                mock.call(
                    _CheckCommand(self, "download", ["setuptools"], []),
                    env=_CheckPythonhomeEnv(self, part_pythonhome),
                    cwd=None,
                ),
                mock.call(
                    _CheckCommand(
                        self, "install", ["setuptools"], ["--ignore-installed"]
                    ),
                    env=_CheckPythonhomeEnv(self, part_pythonhome),
                    cwd=None,
                ),
            ]
        )

    def test_setup_unexpected_error(self):
        """Test that pip initialization doesn't eat legit errors"""

        # Raises an exception indicating something bad happened
        self.mock_run_output.side_effect = subprocess.CalledProcessError(
            1, "foo", b"no good, very bad"
        )

        pip = _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        )

        # Verify that pip lets that exception through
        self.assertRaises(subprocess.CalledProcessError, pip.setup)


class PipBaseTestCase(PythonBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch.object(_pip.Pip, "_run")
        self.mock_run = patcher.start()
        self.addCleanup(patcher.stop)

        self._create_python_binary("install_dir")


class PipTestCase(PipBaseTestCase):
    def test_clean_packages(self):
        pip = _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        )

        packages_dir = os.path.join("part_dir", "python-packages")
        self.assertTrue(os.path.exists(packages_dir))

        # Now verify that asking pip to clean removes its packages
        pip.clean_packages()
        self.assertFalse(os.path.exists(packages_dir))


class PipCommandBaseTestCase(PipBaseTestCase):
    def setUp(self):
        super().setUp()

        self.pip = _pip.Pip(
            python_major_version="test",
            part_dir="part_dir",
            install_dir="install_dir",
            stage_dir="stage_dir",
        )

        # We don't care about anything init did to the mock here: reset it
        self.mock_run.reset_mock()


class PipDownloadTestCase(PipCommandBaseTestCase):

    scenarios = [
        (
            "packages",
            {
                "packages": ["foo", "bar"],
                "kwargs": {},
                "expected_args": ["foo", "bar"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "setup_py_dir",
            {
                "packages": [],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
        (
            "single constraint",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint"]},
                "expected_args": ["--constraint", "constraint"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple constraints",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint1", "constraint2"]},
                "expected_args": [
                    "--constraint",
                    "constraint1",
                    "--constraint",
                    "constraint2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "single requirement",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement"]},
                "expected_args": ["--requirement", "requirement"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple requirements",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement1", "requirement2"]},
                "expected_args": [
                    "--requirement",
                    "requirement1",
                    "--requirement",
                    "requirement2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "process dependency links",
            {
                "packages": [],
                "kwargs": {"process_dependency_links": True},
                "expected_args": ["--process-dependency-links"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "packages and setup_py_dir",
            {
                "packages": ["foo", "bar"],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["foo", "bar", "."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
    ]

    def _assert_mock_run_with(self, *args, **kwargs):
        common_args = ["download", "--disable-pip-version-check", "--dest", mock.ANY]
        common_args.extend(*args)
        self.mock_run.assert_called_once_with(common_args, **kwargs)

    def test_without_packages_or_kwargs_should_noop(self):
        self.pip.download([])
        self.mock_run.assert_not_called()

    def test_with_packages_and_kwargs(self):
        self.pip.download(self.packages, **self.kwargs)
        self._assert_mock_run_with(self.expected_args, **self.expected_kwargs)


class PipInstallTestCase(PipCommandBaseTestCase):

    scenarios = [
        (
            "packages",
            {
                "packages": ["foo", "bar"],
                "kwargs": {},
                "expected_args": ["foo", "bar"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "setup_py_dir",
            {
                "packages": [],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
        (
            "single constraint",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint"]},
                "expected_args": ["--constraint", "constraint"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple constraints",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint1", "constraint2"]},
                "expected_args": [
                    "--constraint",
                    "constraint1",
                    "--constraint",
                    "constraint2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "single requirement",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement"]},
                "expected_args": ["--requirement", "requirement"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple requirements",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement1", "requirement2"]},
                "expected_args": [
                    "--requirement",
                    "requirement1",
                    "--requirement",
                    "requirement2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "process dependency links",
            {
                "packages": [],
                "kwargs": {"process_dependency_links": True},
                "expected_args": ["--process-dependency-links"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "upgrade",
            {
                "packages": [],
                "kwargs": {"upgrade": True},
                "expected_args": ["--upgrade"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "install_deps",
            {
                "packages": [],
                "kwargs": {"install_deps": False},
                "expected_args": ["--no-deps"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "ignore_installed",
            {
                "packages": [],
                "kwargs": {"ignore_installed": True},
                "expected_args": ["--ignore-installed"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "packages and setup_py_dir",
            {
                "packages": ["foo", "bar"],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["foo", "bar", "."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
    ]

    def _assert_mock_run_with(self, *args, **kwargs):
        common_args = [
            "install",
            "--user",
            "--no-compile",
            "--no-index",
            "--find-links",
            mock.ANY,
        ]
        common_args.extend(*args)
        self.mock_run.assert_called_once_with(common_args, **kwargs)

    def test_without_packages_or_kwargs_should_noop(self):
        self.pip.install([])
        self.mock_run.assert_not_called()

    def test_with_packages_and_kwargs(self):
        self.pip.install(self.packages, **self.kwargs)
        self._assert_mock_run_with(self.expected_args, **self.expected_kwargs)


class PipInstallFixupShebangTestCase(PipCommandBaseTestCase):

    scenarios = [
        (
            "bad shebang",
            {
                "file_path": "example.py",
                "contents": "#!/foo/bar/baz/python",
                "expected": "#!/usr/bin/env python",
            },
        ),
        (
            "bad shebang and bad permissions",
            {
                "file_path": os.path.join("bin", "another_example.py"),
                "contents": "#!/foo/baz/python3",
                "expected": "#!/usr/bin/env python3",
            },
        ),
        (
            "no shebang",
            {
                "file_path": "foo",
                "contents": "foo\n#!/usr/bin/python3",
                "expected": "foo\n#!/usr/bin/python3",
            },
        ),
        (
            "no shebang and bad permissions",
            {"file_path": "bar", "contents": "bar", "expected": "bar"},
        ),
        (
            "good shebang",
            {
                "file_path": "baz",
                "contents": "#!/usr/bin/env python",
                "expected": "#!/usr/bin/env python",
            },
        ),
        (
            "good shebang and bad permissions",
            {
                "file_path": os.path.join("qux", "another_example.py"),
                "contents": "#!/usr/bin/env python",
                "expected": "#!/usr/bin/env python",
            },
        ),
    ]

    def setUp(self):
        super().setUp()

        self.file_path = os.path.join(self.pip._install_dir, self.file_path)
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        with open(self.file_path, "w") as f:
            f.write(self.contents)

        self.pip.install(["foo"])

    def test_install_fixes_shebangs(self):
        with open(self.file_path, "r") as f:
            self.assertThat(f.read(), Equals(self.expected))


class PipInstallFixupPermissionsTestCase(PipCommandBaseTestCase):

    scenarios = [
        ("755", {"file_path": "example.py", "mode": 0o755, "expected_mode": "755"}),
        (
            "500",
            {
                "file_path": os.path.join("bin", "another_example.py"),
                "mode": 0o500,
                "expected_mode": "755",
            },
        ),
        ("777", {"file_path": "foo", "mode": 0o777, "expected_mode": "777"}),
        ("700", {"file_path": "bar", "mode": 0o700, "expected_mode": "755"}),
        (
            "000",
            {
                "file_path": os.path.join("qux", "another_example.py"),
                "mode": 0o000,
                "expected_mode": "200",
            },
        ),
    ]

    def setUp(self):
        super().setUp()

        self.file_path = os.path.join(self.pip._install_dir, self.file_path)
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        with open(self.file_path, "w") as f:
            f.write("bubbles!")

        os.chmod(self.file_path, self.mode)

        self.pip.install(["foo"])

    def test_install_fixes_permissions(self):
        self.assertThat(
            oct(os.stat(self.file_path).st_mode)[-3:], Equals(self.expected_mode)
        )

    @mock.patch.object(os, "stat")
    @mock.patch.object(os.path, "exists", return_value=False)
    def test_missing_path(self, mock_path_exists, mock_os_stat):
        _pip._replicate_owner_mode("/nonexistant_path")
        self.assertFalse(mock_os_stat.called)

    @mock.patch.object(os, "chmod")
    def test_symlink(self, mock_chmod):
        os.symlink(self.file_path, "link")
        _pip._replicate_owner_mode("link")
        self.assertFalse(mock_chmod.called)


class PipWheelTestCase(PipCommandBaseTestCase):

    scenarios = [
        (
            "packages",
            {
                "packages": ["foo", "bar"],
                "kwargs": {},
                "expected_args": ["foo", "bar"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "setup_py_dir",
            {
                "packages": [],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
        (
            "single constraint",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint"]},
                "expected_args": ["--constraint", "constraint"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple constraints",
            {
                "packages": [],
                "kwargs": {"constraints": ["constraint1", "constraint2"]},
                "expected_args": [
                    "--constraint",
                    "constraint1",
                    "--constraint",
                    "constraint2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "single requirement",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement"]},
                "expected_args": ["--requirement", "requirement"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "multiple requirements",
            {
                "packages": [],
                "kwargs": {"requirements": ["requirement1", "requirement2"]},
                "expected_args": [
                    "--requirement",
                    "requirement1",
                    "--requirement",
                    "requirement2",
                ],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "process dependency links",
            {
                "packages": [],
                "kwargs": {"process_dependency_links": True},
                "expected_args": ["--process-dependency-links"],
                "expected_kwargs": {"cwd": None},
            },
        ),
        (
            "packages and setup_py_dir",
            {
                "packages": ["foo", "bar"],
                "kwargs": {"setup_py_dir": "test_setup_py_dir"},
                "expected_args": ["foo", "bar", "."],
                "expected_kwargs": {"cwd": "test_setup_py_dir"},
            },
        ),
    ]

    def _assert_mock_run_with(self, *args, **kwargs):
        common_args = [
            "wheel",
            "--no-index",
            "--find-links",
            mock.ANY,
            "--wheel-dir",
            mock.ANY,
        ]
        common_args.extend(*args)
        self.mock_run.assert_called_once_with(common_args, **kwargs)

    def test_without_packages_or_kwargs_should_noop(self):
        self.pip.wheel([])
        self.mock_run.assert_not_called()

    def test_with_packages_and_kwargs(self):
        self.pip.wheel(self.packages, **self.kwargs)
        self._assert_mock_run_with(self.expected_args, **self.expected_kwargs)


class PipListTestCase(PipCommandBaseTestCase):
    def test_none(self):
        self.mock_run.return_value = "{}"
        self.assertFalse(self.pip.list())
        self.mock_run.assert_called_once_with(
            ["list", "--format=json"], runner=mock.ANY
        )

    def test_package(self):
        self.mock_run.return_value = '[{"name": "foo", "version": "1.0"}]'
        self.assertThat(self.pip.list(), Equals({"foo": "1.0"}))
        self.mock_run.assert_called_once_with(
            ["list", "--format=json"], runner=mock.ANY
        )

    def test_user(self):
        self.mock_run.return_value = '[{"name": "foo", "version": "1.0"}]'
        self.assertThat(self.pip.list(user=True), Equals({"foo": "1.0"}))
        self.mock_run.assert_called_once_with(
            ["list", "--user", "--format=json"], runner=mock.ANY
        )

    def test_missing_name(self):
        self.mock_run.return_value = '[{"version": "1.0"}]'
        raised = self.assertRaises(errors.PipListMissingFieldError, self.pip.list)
        self.assertThat(str(raised), Contains("Pip packages json missing 'name' field"))

    def test_missing_version(self):
        self.mock_run.return_value = '[{"name": "foo"}]'
        raised = self.assertRaises(errors.PipListMissingFieldError, self.pip.list)
        self.assertThat(
            str(raised), Contains("Pip packages json missing 'version' field")
        )

    def test_invalid_json(self):
        self.mock_run.return_value = "[{]"
        raised = self.assertRaises(errors.PipListInvalidJsonError, self.pip.list)
        self.assertThat(str(raised), Contains("Pip packages output isn't valid json"))

    def test_no_json_format(self):
        self.mock_run.side_effect = [
            subprocess.CalledProcessError(2, "pip", "no such option"),
            "foo (1.0)",
        ]
        self.assertThat(self.pip.list(), Equals({"foo": "1.0"}))
        self.mock_run.assert_has_calls(
            [
                mock.call(["list", "--format=json"], runner=mock.ANY),
                mock.call(["list"], runner=mock.ANY),
            ]
        )

    def test_no_json_format_invalid_result(self):
        self.mock_run.side_effect = [
            subprocess.CalledProcessError(2, "pip", "no such option"),
            "foo 1.0",
        ]
        raised = self.assertRaises(
            errors.PipListInvalidLegacyFormatError, self.pip.list
        )
        self.assertThat(raised.output, Equals("foo 1.0"))


class _CheckPythonhomeEnv:
    def __init__(self, test, expected_pythonhome):
        self.test = test
        self.expected_pythonhome = expected_pythonhome

    def __eq__(self, env):
        # Verify that we're using the installed pip
        self.test.assertThat(env, Contains("PYTHONHOME"))
        self.test.assertThat(env["PYTHONHOME"], Equals(self.expected_pythonhome))

        return True


class _CheckCommand:
    def __init__(self, test, command, packages, flags):
        self.test = test
        self.command = command
        self.packages = packages
        self.flags = flags

    def __eq__(self, command):
        # Not worrying about the command arguments here, those are
        # tested elsewhere. Just want to test that the right command
        # is called with the right packages.
        self.test.assertTrue(command)
        self.test.assertThat(command[len(self.test.command)], Equals(self.command))

        for package in self.packages:
            self.test.assertThat(command, Contains(package))

        for flag in self.flags:
            self.test.assertThat(command, Contains(flag))

        return True
