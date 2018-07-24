# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import json
import logging
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import List, Set

import snapcraft
from snapcraft import file_utils
from snapcraft.internal import mangling
from ._python_finder import get_python_command, get_python_headers, get_python_home
from . import errors

logger = logging.getLogger(__name__)


def _process_common_args(
    *,
    packages: List[str],
    constraints: Set[str],
    requirements: Set[str],
    process_dependency_links: bool
) -> List[str]:
    args = []
    if constraints:
        for constraint in constraints:
            args.extend(["--constraint", constraint])

    if requirements:
        for requirement in requirements:
            args.extend(["--requirement", requirement])

    if process_dependency_links:
        args.append("--process-dependency-links")

    if packages:
        args.extend(packages)

    return args


def _replicate_owner_mode(path):
    # Don't bother with a path that doesn't exist or is a symlink. The target
    # of the symlink will either be updated anyway, or we won't have permission
    # to change it.
    if not os.path.exists(path) or os.path.islink(path):
        return

    file_mode = os.stat(path).st_mode

    # We at least need to write to it to fix shebangs later
    new_mode = file_mode | stat.S_IWUSR

    # If the owner can execute it, so should everyone else.
    if file_mode & stat.S_IXUSR:
        new_mode |= stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH

    # If the owner can read it, so should everyone else
    if file_mode & stat.S_IRUSR:
        new_mode |= stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH

    os.chmod(path, new_mode)


def _fix_permissions(path):
    for root, dirs, files in os.walk(path):
        for filename in files:
            _replicate_owner_mode(os.path.join(root, filename))
        for dirname in dirs:
            _replicate_owner_mode(os.path.join(root, dirname))


class Pip:
    """Wrapper for pip abstracting the args necessary for use in a part.

    This class takes care of fetching pip, setuptools, and wheel, and then
    simply shells out to pip with the magical arguments necessary to install
    packages into a part.

    Of particular importance: packages must be downloaded (via download())
    before they can be installed or have wheels built.
    """

    def __init__(self, *, python_major_version, part_dir, install_dir, stage_dir):
        """Initialize pip.

        You must call setup() before you can actually use pip.

        :param str python_major_version: The python major version to find (2 or
                                         3)
        :param str part_dir: Path to the part's working area
        :param str install_dir: Path to the part's install area
        :param str stage_dir: Path to the staging area

        :raises MissingPythonCommandError: If no python could be found in the
                                           staging or part's install area.
        """
        self._python_major_version = python_major_version
        self._install_dir = install_dir
        self._stage_dir = stage_dir

        self._python_package_dir = os.path.join(part_dir, "python-packages")
        os.makedirs(self._python_package_dir, exist_ok=True)

        self.__python_command = None  # type:str
        self.__python_home = None  # type: str

    @property
    def _python_command(self):
        """Lazily determine the python command required."""
        if not self.__python_command:
            self.__python_command = get_python_command(
                self._python_major_version,
                stage_dir=self._stage_dir,
                install_dir=self._install_dir,
            )
        return self.__python_command

    @property
    def _python_home(self):
        """Lazily determine the correct python home."""
        if not self.__python_home:
            self.__python_home = get_python_home(
                self._python_major_version,
                stage_dir=self._stage_dir,
                install_dir=self._install_dir,
            )
        return self.__python_home

    def setup(self):
        """Install pip and dependencies.

        Check to see if pip has already been installed. If not, fetch pip,
        setuptools, and wheel, and install them so they can be used.
        """

        self._ensure_pip_installed()
        self._ensure_wheel_installed()
        self._ensure_setuptools_installed()

    def is_setup(self):
        """Return true if this class has already been setup."""

        return (
            self._is_pip_installed()
            and self._is_wheel_installed()
            and self._is_setuptools_installed()
        )

    def _ensure_pip_installed(self):
        # Check to see if we have our own pip. If not, we need to use the pip
        # on the host (installed via build-packages) to grab our own.
        if not self._is_pip_installed():
            logger.info("Fetching and installing pip...")

            real_python_home = self.__python_home

            # Make sure we're using pip from the host. Wrapping this operation
            # in a try/finally to make sure we revert away from the host's
            # python at the end.
            try:
                self.__python_home = os.path.join(os.path.sep, "usr")

                # Using the host's pip, install our own pip
                self.download({"pip"})
                self.install({"pip"}, ignore_installed=True)
            finally:
                # Now that we have our own pip, reset the python home
                self.__python_home = real_python_home

    def _ensure_wheel_installed(self):
        if not self._is_wheel_installed():
            logger.info("Fetching and installing wheel...")
            self.download({"wheel"})
            self.install({"wheel"}, ignore_installed=True)

    def _ensure_setuptools_installed(self):
        if not self._is_setuptools_installed():
            logger.info("Fetching and installing setuptools...")
            self.download({"setuptools"})
            self.install({"setuptools"}, ignore_installed=True)

    def _is_pip_installed(self):
        try:
            # We're expecting an error here at least once complaining about
            # pip not being installed. In order to verify that the error is the
            # one we think it is, we need to process the stderr. So we'll
            # redirect it to stdout. If it's not the error we expect, something
            # is wrong, so re-raise it.
            #
            # Using _run_output here so stdout doesn't get printed to the
            # terminal.
            self._run_output([], stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            if "no module named pip" in output.lower():
                return False
            else:
                raise e
        return True

    def _is_wheel_installed(self):
        return "wheel" in self.list()

    def _is_setuptools_installed(self):
        return "setuptools" in self.list()

    def download(
        self,
        packages,
        *,
        setup_py_dir=None,
        constraints=None,
        requirements=None,
        process_dependency_links=False
    ):
        """Download packages into cache, but do not install them.

        :param iterable packages: Packages to download from index.
        :param str setup_py_dir: Directory containing setup.py.
        :param iterable constraints: Collection of paths to constraints files.
        :param iterable requirements: Collection of paths to requirements
                                      files.
        :param boolean process_dependency_links: Enable the processing of
                                                 dependency links.
        """
        args = _process_common_args(
            process_dependency_links=process_dependency_links,
            packages=packages,
            constraints=constraints,
            requirements=requirements,
        )

        cwd = None
        if setup_py_dir:
            args.append(".")
            cwd = setup_py_dir

        if not args:
            return  # No operation was requested

        # Using pip with a few special parameters:
        #
        # --disable-pip-version-check: Don't whine if pip is out-of-date with
        #                              the version on pypi.
        # --dest: Download packages into the directory we've set aside for it.
        self._run(
            [
                "download",
                "--disable-pip-version-check",
                "--dest",
                self._python_package_dir,
            ]
            + args,
            cwd=cwd,
        )

    def install(
        self,
        packages,
        *,
        setup_py_dir=None,
        constraints=None,
        requirements=None,
        process_dependency_links=False,
        upgrade=False,
        install_deps=True,
        ignore_installed=False
    ):
        """Install packages from cache.

        The packages should have already been downloaded via `download()`.

        :param iterable packages: Packages to install from cache.
        :param str setup_py_dir: Directory containing setup.py.
        :param iterable constraints: Collection of paths to constraints files.
        :param iterable requirements: Collection of paths to requirements
                                      files.
        :param boolean process_dependency_links: Enable the processing of
                                                 dependency links.
        :param boolean upgrade: Recursively upgrade packages.
        :param boolean install_deps: Install package dependencies.
        :param boolean ignore_installed: Reinstall packages if they're already
                                         installed
        """
        args = _process_common_args(
            process_dependency_links=process_dependency_links,
            packages=packages,
            constraints=constraints,
            requirements=requirements,
        )

        if upgrade:
            args.append("--upgrade")

        if not install_deps:
            args.append("--no-deps")

        if ignore_installed:
            args.append("--ignore-installed")

        cwd = None
        if setup_py_dir:
            args.append(".")
            cwd = setup_py_dir

        if not args:
            return  # No operation was requested

        # Using pip with a few special parameters:
        #
        # --user: Install packages to PYTHONUSERBASE, which we've pointed to
        #         the installdir.
        # --no-compile: Don't compile .pyc files. FIXME: This is legacy, and
        #               should be removed once this refactor has been
        #               validated.
        # --no-index: Don't hit pypi, assume the packages are already
        #             downloaded (i.e. by using `self.download()`)
        # --find-links: Provide the directory into which the packages should
        #               have already been fetched
        self._run(
            [
                "install",
                "--user",
                "--no-compile",
                "--no-index",
                "--find-links",
                self._python_package_dir,
            ]
            + args,
            cwd=cwd,
        )

        # Installing with --user results in a directory with 700 permissions.
        # We need it a bit more open than that, so open it up.
        _fix_permissions(self._install_dir)

        # Fix all shebangs to use the in-snap python.
        mangling.rewrite_python_shebangs(self._install_dir)

    def wheel(
        self,
        packages,
        *,
        setup_py_dir=None,
        constraints=None,
        requirements=None,
        process_dependency_links=False
    ):
        """Build wheels of packages in the cache.

        The packages should have already been downloaded via `download()`.

        :param iterable packages: Packages in cache for which to build wheels.
        :param str setup_py_dir: Directory containing setup.py.
        :param iterable constraints: Collection of paths to constraints files.
        :param iterable requirements: Collection of paths to requirements
                                      files.
        :param boolean process_dependency_links: Enable the processing of
                                                 dependency links.

        :return: List of paths to each wheel that was built.
        :rtype: list
        """
        args = _process_common_args(
            process_dependency_links=process_dependency_links,
            packages=packages,
            constraints=constraints,
            requirements=requirements,
        )

        cwd = None
        if setup_py_dir:
            args.append(".")
            cwd = setup_py_dir

        if not args:
            return []  # No operation was requested

        wheels = []
        with tempfile.TemporaryDirectory() as temp_dir:

            # Using pip with a few special parameters:
            #
            # --no-index: Don't hit pypi, assume the packages are already
            #             downloaded (i.e. by using `self.download()`)
            # --find-links: Provide the directory into which the packages
            #               should have already been fetched
            # --wheel-dir: Build wheels into a temporary working area rather
            #              rather than cwd. We'll copy them over. FIXME: We can
            #              probably get away just building them in the package
            #              dir. Try that once this refactor has been validated.
            self._run(
                [
                    "wheel",
                    "--no-index",
                    "--find-links",
                    self._python_package_dir,
                    "--wheel-dir",
                    temp_dir,
                ]
                + args,
                cwd=cwd,
            )
            wheels = os.listdir(temp_dir)
            for wheel in wheels:
                file_utils.link_or_copy(
                    os.path.join(temp_dir, wheel),
                    os.path.join(self._python_package_dir, wheel),
                )

        return [os.path.join(self._python_package_dir, wheel) for wheel in wheels]

    def list(self, *, user=False):
        """Determine which packages have been installed.

        :param boolean user: Whether or not to limit results to user base.

        :return: Dict of installed python packages and their versions
        :rtype: dict
        """
        command = ["list"]
        if user:
            command.append("--user")

        packages = collections.OrderedDict()
        try:
            output = self._run_output(command + ["--format=json"])
            json_output = json.loads(output, object_pairs_hook=collections.OrderedDict)
        except subprocess.CalledProcessError:
            # --format requires a newer pip, so fall back to legacy output
            output = self._run_output(command)
            json_output = []  # type: List[Dict[str, str]]
            version_regex = re.compile("\((.+)\)")
            for line in output.splitlines():
                line = line.split()
                m = version_regex.search(line[1])
                if not m:
                    raise errors.PipListInvalidLegacyFormatError(output)
                json_output.append({"name": line[0], "version": m.group(1)})
        except json.decoder.JSONDecodeError as e:
            raise errors.PipListInvalidJsonError(output) from e

        for package in json_output:
            if "name" not in package:
                raise errors.PipListMissingFieldError("name", output)
            if "version" not in package:
                raise errors.PipListMissingFieldError("version", output)
            packages[package["name"]] = package["version"]
        return packages

    def clean_packages(self):
        """Remove the package cache."""
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._python_package_dir)

    def env(self):
        """The environment used by pip.

        This function is only useful if you happen to need to call into pip's
        environment without using the API otherwise made available here (e.g.
        calling the setup.py directly instead of with pip).

        :return: Dict of the environment necessary to use the pip contained
                 here.
        :rtype: dict
        """
        env = os.environ.copy()
        env["PYTHONUSERBASE"] = self._install_dir
        env["PYTHONHOME"] = self._python_home

        env["PATH"] = "{}:{}".format(
            os.path.join(self._install_dir, "usr", "bin"), os.path.expandvars("$PATH")
        )

        headers = get_python_headers(
            self._python_major_version, stage_dir=self._stage_dir
        )
        if headers:
            current_cppflags = env.get("CPPFLAGS", "")
            env["CPPFLAGS"] = "-I{}".format(headers)
            if current_cppflags:
                env["CPPFLAGS"] = "{} {}".format(env["CPPFLAGS"], current_cppflags)

        return env

    def _run(self, args, runner=None, **kwargs):
        env = self.env()

        # Using None as the default value instead of common.run so we can mock
        # common.run.
        if runner is None:
            runner = snapcraft.internal.common.run

        return runner(
            [self._python_command, "-m", "pip"] + list(args), env=env, **kwargs
        )

    def _run_output(self, args, **kwargs):
        return self._run(args, runner=snapcraft.internal.common.run_output, **kwargs)
