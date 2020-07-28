# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

import distutils.util
import glob
import os
import shutil
import subprocess
import sys
from distutils import dir_util
from textwrap import dedent
from typing import Callable, List, Union

import fixtures
import requests
import testtools
import yaml as yaml_utils
from pexpect import popen_spawn
from testtools import content

from tests import fixture_setup, os_release, subprocess_utils
from tests.integration import platform


class RegisterError(Exception):
    pass


class TestCase(testtools.TestCase):
    def setUp(self):
        super().setUp()

        self.patchelf_command = "patchelf"
        self.execstack_command = "execstack"

        package_type = os.getenv("SNAPCRAFT_PACKAGE_TYPE")
        if package_type == "snap":
            self.snapcraft_command = "/snap/bin/snapcraft"
            self.patchelf_command = "/snap/snapcraft/current/bin/patchelf"
            self.execstack_command = "/snap/snapcraft/current/usr/bin/execstack"
        elif package_type == "deb":
            self.snapcraft_command = "/usr/bin/snapcraft"
            self.snapcraft_parser_command = "/usr/bin/snapcraft-parser"
        elif os.getenv("VIRTUAL_ENV") and sys.platform == "win32":
            self.snapcraft_command = os.path.join(
                os.getenv("VIRTUAL_ENV"), "Scripts", "snapcraft.exe"
            )
            self.snapcraft_parser_command = os.path.join(
                os.getenv("VIRTUAL_ENV"), "Scripts", "snapcraft-parser.exe"
            )
        elif os.getenv("VIRTUAL_ENV"):
            self.snapcraft_command = os.path.join(
                os.getenv("VIRTUAL_ENV"), "bin", "snapcraft"
            )
            self.snapcraft_parser_command = os.path.join(
                os.getenv("VIRTUAL_ENV"), "bin", "snapcraft-parser"
            )
        elif package_type == "brew":
            self.snapcraft_command = "/usr/local/bin/snapcraft"
        else:
            raise EnvironmentError(
                "snapcraft is not setup correctly for testing. Either set "
                "SNAPCRAFT_PACKAGE_TYPE to 'snap', 'deb' or 'brew', to run from "
                "either the snap, deb or homebrew or make sure your venv is properly "
                "setup as described in HACKING.md."
            )

        self.snaps_dir = os.path.join(os.path.dirname(__file__), "snaps")
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        # Use a separate path for XDG dirs, or changes there may be detected as
        # source changes.
        self.xdg_path = self.useFixture(fixtures.TempDir()).path
        self.useFixture(fixture_setup.TempXDG(self.xdg_path))

        # Use this host to run through the lifecycle tests
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )

        # Use a dumb terminal for tests
        self.useFixture(fixtures.EnvironmentVariable("TERM", "dumb"))

        # Force appearance of TTY for pexpect to work with echo.is_connected_tty()
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_HAS_TTY", "y"))

        # Disable Sentry reporting for tests, otherwise they'll hang waiting
        # for input
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_ERROR_REPORTING", "false")
        )

        # Don't let the managed host variable leak into tests
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_MANAGED_HOST"))

        # Note that these directories won't exist when the test starts,
        # they might be created after calling the snapcraft command on the
        # project dir.
        self.parts_dir = "parts"
        self.stage_dir = "stage"
        self.prime_dir = "prime"

        self.deb_arch = platform.get_deb_arch()
        self.arch_triplet = platform.get_arch_triplet()

        self.distro_series = os_release.get_version_codename()

    def run_snapcraft(
        self,
        command: Union[str, List[str]] = None,
        project_dir: str = None,
        debug: bool = True,
        pre_func: Callable[[], None] = lambda: None,
        env=None,
    ) -> str:
        if project_dir:
            self.copy_project_to_cwd(project_dir)

        if command is None:
            command = []
        if isinstance(command, str):
            command = [command]
        snapcraft_command = self.snapcraft_command
        if isinstance(snapcraft_command, str):
            snapcraft_command = [snapcraft_command]
        if debug:
            snapcraft_command.append("-d")
        try:
            pre_func()
            snapcraft_output = subprocess.check_output(
                snapcraft_command + command,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                env=env,
            )
        except subprocess.CalledProcessError as e:
            self.addDetail("command", content.text_content(str(self.snapcraft_command)))
            self.addDetail("output", content.text_content(e.output))
            raise
        except FileNotFoundError:
            self.addDetail("command", content.text_content(str(self.snapcraft_command)))
            raise

        if not os.getenv("SNAPCRAFT_IGNORE_APT_AUTOREMOVE", False):
            self.addCleanup(self.run_apt_autoremove)

        return snapcraft_output

    def spawn_snapcraft(self, command: Union[str, List[str]]):
        snapcraft_command = self.snapcraft_command
        if isinstance(snapcraft_command, str):
            snapcraft_command = [snapcraft_command]
        try:
            return popen_spawn.PopenSpawn(" ".join(snapcraft_command + command))
        except FileNotFoundError:
            self.addDetail("command", content.text_content(str(snapcraft_command)))

    def run_snapcraft_parser(self, arguments):
        try:
            snapcraft_output = subprocess.check_output(
                [self.snapcraft_parser_command, "-d"] + arguments,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
            )
        except subprocess.CalledProcessError as e:
            self.addDetail("output", content.text_content(e.output))
            raise
        return snapcraft_output

    def run_apt_autoremove(self):
        if sys.platform == "win32":
            return

        deb_env = os.environ.copy()
        deb_env.update(
            {"DEBIAN_FRONTEND": "noninteractive", "DEBCONF_NONINTERACTIVE_SEEN": "true"}
        )

        try:
            autoremove_output = subprocess.check_output(
                "sudo apt-get autoremove -y".split(),
                stderr=subprocess.STDOUT,
                env=deb_env,
            )
            self.addDetail(
                "apt-get autoremove output",
                content.text_content(autoremove_output.decode("utf-8")),
            )
        except FileNotFoundError as e:
            self.addDetail("apt-get autoremove error", content.text_content(str(e)))
        except subprocess.CalledProcessError as e:
            self.addDetail("apt-get autoremove error", content.text_content(str(e)))
            self.addDetail(
                "apt-get autoremove output",
                content.text_content(e.output.decode("utf-8")),
            )

            if os.getenv("SNAPCRAFT_APT_AUTOREMOVE_CHECK_FAIL", False):
                raise

    def copy_project_to_cwd(self, project_dir: str) -> None:
        # Because cwd already exists, shutil.copytree would raise
        # FileExistsError. Use the lesser known distutils.dir_util.copy_tree
        dir_util.copy_tree(
            os.path.join(self.snaps_dir, project_dir), self.path, preserve_symlinks=True
        )

    def construct_yaml(
        self,
        name="test",
        version="0.1",
        summary="Simple test snap",
        description="Something something",
        grade=None,
        architectures=None,
        parts=dedent(
            """\
                           my-part:
                             plugin: nil
                           """
        ),
        build_packages="[]",
        adopt_info=None,
    ):
        snapcraft_yaml = {
            "name": name,
            "summary": summary,
            "description": description,
            "parts": yaml_utils.load(parts),
            "build-packages": yaml_utils.load(build_packages),
        }

        if version:
            snapcraft_yaml["version"] = version
        if adopt_info:
            snapcraft_yaml["adopt-info"] = adopt_info
        if grade:
            snapcraft_yaml["grade"] = grade
        if architectures:
            snapcraft_yaml["architectures"] = architectures

        with open("snapcraft.yaml", "w") as f:
            yaml_utils.dump(snapcraft_yaml, stream=f)

    def get_output_ignoring_non_zero_exit(self, binary, cwd=None):
        # Executing the binaries exists > 0 on trusty.
        # TODO investigate more to understand the cause.
        try:
            output = subprocess.check_output(binary, universal_newlines=True, cwd=cwd)
        except subprocess.CalledProcessError as exception:
            output = exception.output
        return output

    def set_stage_package_version(
        self, snapcraft_yaml_path, part, package, version=None
    ):
        return self.set_package_version(
            "stage-packages", snapcraft_yaml_path, part, package, version
        )

    def set_build_package_version(
        self, snapcraft_yaml_path, part, package, version=None
    ):
        return self.set_package_version(
            "build-packages", snapcraft_yaml_path, part, package, version
        )

    def set_package_version(
        self, type_, snapcraft_yaml_path, part, package, version=None
    ):
        # This doesn't handle complex package syntax.
        with open(snapcraft_yaml_path) as snapcraft_yaml_file:
            snapcraft_yaml = yaml_utils.load(snapcraft_yaml_file)
        if part:
            packages = snapcraft_yaml["parts"][part].get(type_, [])
        else:
            packages = snapcraft_yaml.get(type_, [])
        for index, package_in_yaml in enumerate(packages):
            if package_in_yaml.split("=")[0] == package:
                if version is None:
                    version = get_package_version(
                        package, self.distro_series, self.deb_arch
                    )

                packages[index] = "{}={}".format(package, version)
                break
        else:
            self.fail("The part {} doesn't have a package {}".format(part, package))

        with open(snapcraft_yaml_path, "w") as snapcraft_yaml_file:
            yaml_utils.dump(snapcraft_yaml, stream=snapcraft_yaml_file)
        return version

    def set_build_package_architecture(
        self, snapcraft_yaml_path, part, package, architecture
    ):
        # This doesn't handle complex package syntax.
        with open(snapcraft_yaml_path) as snapcraft_yaml_file:
            snapcraft_yaml = yaml_utils.load(snapcraft_yaml_file)
        packages = snapcraft_yaml["parts"][part]["build-packages"]
        for index, package_in_yaml in enumerate(packages):
            if package_in_yaml == package:
                packages[index] = "{}:{}".format(package, architecture)
                break
        else:
            self.fail("The part {} doesn't have a package {}".format(part, package))

        with open(snapcraft_yaml_path, "w") as snapcraft_yaml_file:
            yaml_utils.dump(snapcraft_yaml, stream=snapcraft_yaml_file)


class BzrSourceBaseTestCase(TestCase):
    def setUp(self):
        super().setUp()
        if shutil.which("bzr") is None:
            self.skipTest("bzr is not installed")

    def init_source_control(self):
        subprocess.check_call(
            ["bzr", "init", "."], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        subprocess.check_call(
            ["bzr", "whoami", "--branch", '"Example Dev <dev@example.com>"']
        )

    def commit(self, message, unchanged=False):
        command = ["bzr", "commit", "-m", message]
        if unchanged:
            command.append("--unchanged")
        subprocess.check_call(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def get_revno(self, path=None):
        command = ["bzr", "revno", "-r", "-1"]
        if path:
            command.append(path)
        return subprocess.check_output(command, universal_newlines=True).strip()


class GitSourceBaseTestCase(TestCase):
    def setUp(self):
        super().setUp()
        if shutil.which("git") is None:
            self.skipTest("git is not installed")

    def init_source_control(self):
        subprocess.check_call(["git", "init", "."], stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ["git", "config", "--local", "user.name", '"Example Dev"']
        )
        subprocess.check_call(
            ["git", "config", "--local", "user.email", "dev@example.com"]
        )

    def add_file(self, file_path):
        subprocess.check_call(["git", "add", file_path], stdout=subprocess.DEVNULL)

    def commit(self, message, allow_empty=False):
        command = ["git", "commit", "-m", message]
        if allow_empty:
            command.append("--allow-empty")
        subprocess.check_call(command, stdout=subprocess.DEVNULL)

    def tag(self, tag_name):
        subprocess.check_call(
            ["git", "tag", "-a", "-m", tag_name, tag_name], stdout=subprocess.DEVNULL
        )

    def get_revno(self):
        return subprocess_utils.call_with_output(
            ["git", "rev-list", "HEAD", "--max-count=1"]
        )


class HgSourceBaseTestCase(TestCase):
    def setUp(self):
        super().setUp()
        if shutil.which("hg") is None:
            self.skipTest("mercurial is not installed")

    def init_source_control(self):
        subprocess.check_call(["hg", "init", "."])

    def commit(self, message, file_):
        subprocess.check_call(
            ["hg", "commit", "-m", message, "--user", '"Example Dev"', "-A", file_]
        )

    def get_revno(self, path=None):
        command = ["hg", "log", "--template", '"{desc}"', "-r", "-1"]
        if path:
            command.extend(["--cwd", path])
        return subprocess.check_output(command, universal_newlines=True).strip()

    def get_id(self):
        return subprocess_utils.call_with_output(["hg", "id"]).split()[0]


class SubversionSourceBaseTestCase(TestCase):
    def setUp(self):
        super().setUp()
        if shutil.which("svn") is None:
            self.skipTest("svn is not installed")

    def init_source_control(self):
        subprocess.check_call(["svnadmin", "create", "repo"], stdout=subprocess.DEVNULL)

    def checkout(self, source, destination):
        subprocess.check_call(
            ["svn", "checkout", source, destination], stdout=subprocess.DEVNULL
        )

    def add(self, file_path, cwd=None):
        subprocess.check_call(
            ["svn", "add", file_path], stdout=subprocess.DEVNULL, cwd=cwd
        )

    def commit(self, message, cwd=None):
        subprocess.check_call(
            ["svn", "commit", "-m", message], stdout=subprocess.DEVNULL, cwd=cwd
        )

    def update(self, cwd=None):
        subprocess.check_call(["svn", "update"], stdout=subprocess.DEVNULL, cwd=cwd)


class SnapdIntegrationTestCase(TestCase):

    slow_test = False

    def setUp(self) -> None:
        super().setUp()
        run_slow_tests = os.environ.get("SNAPCRAFT_SLOW_TESTS", False)
        if run_slow_tests:
            run_slow_tests = distutils.util.strtobool(str(run_slow_tests))
        if self.slow_test and not run_slow_tests:
            self.skipTest("Not running slow tests")
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")

    def install_snap(self) -> None:
        try:
            subprocess.check_output(
                ["sudo", "snap", "install", glob.glob("*.snap")[0], "--dangerous"],
                stderr=subprocess.STDOUT,
                universal_newlines=True,
            )
        except subprocess.CalledProcessError as e:
            self.addDetail("output", content.text_content(e.output))
            raise


def get_package_version(package_name, series, deb_arch):
    # http://people.canonical.com/~ubuntu-archive/madison.cgi?package=hello&a=amd64&c=&s=zesty&text=on
    params = {
        "package": package_name,
        "s": "{0},{0}-updates,{0}-security".format(series),
        "a": deb_arch,
        "text": "on",
    }
    query = requests.get(
        "https://people.canonical.com/~ubuntu-archive/madison.cgi", params
    )
    query.raise_for_status()
    package = query.text.strip().split("\n")[-1]
    package_status = [i.strip() for i in package.strip().split("|")]
    return package_status[1]


def add_stage_packages(
    *, part_name: str, stage_packages: List[str], snapcraft_yaml_file=None
):
    if snapcraft_yaml_file is None:
        snapcraft_yaml_file = os.path.join("snap", "snapcraft.yaml")

    with open(snapcraft_yaml_file) as file_read:
        y = yaml_utils.load(file_read)
        if "stage-packages" in y["parts"][part_name]:
            y["parts"][part_name]["stage-packages"].extend(stage_packages)
        else:
            y["parts"][part_name]["stage-packages"] = stage_packages
    with open(snapcraft_yaml_file, "w") as file_write:
        yaml_utils.dump(y, stream=file_write)
