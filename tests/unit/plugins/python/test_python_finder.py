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
import re

from unittest import mock
from testtools.matchers import Equals, MatchesRegex

from snapcraft.plugins._python import errors, _python_finder

from ._basesuite import PythonBaseTestCase


class GetPythonCommandTestCase(PythonBaseTestCase):
    def test_staged(self):
        """get_python_command should support staged python"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create staged binary
        self._create_python_binary(stage_dir)

        python_command = _python_finder.get_python_command(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(
            python_command, Equals(os.path.join(stage_dir, "usr", "bin", "pythontest"))
        )

    def test_in_part(self):
        """get_python_command should support in-part python"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create installed binary
        self._create_python_binary(install_dir)

        python_command = _python_finder.get_python_command(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(
            python_command,
            Equals(os.path.join(install_dir, "usr", "bin", "pythontest")),
        )

    def test_staged_and_in_part(self):
        """get_python_command should prefer staged python over in-part"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create both staged and installed binaries
        self._create_python_binary(stage_dir)
        self._create_python_binary(install_dir)

        python_command = _python_finder.get_python_command(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(
            python_command, Equals(os.path.join(stage_dir, "usr", "bin", "pythontest"))
        )

    def test_missing_raises(self):
        """get_python_command should raise if no python can be found"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        raised = self.assertRaises(
            errors.MissingPythonCommandError,
            _python_finder.get_python_command,
            "test",
            stage_dir=stage_dir,
            install_dir=install_dir,
        )
        self.assertThat(
            str(raised),
            Equals("Unable to find pythontest, searched: stage_dir:install_dir"),
        )


class GetPythonHeadersTestCase(PythonBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("glob.glob")
        self.mock_glob = patcher.start()
        self.addCleanup(patcher.stop)

    def test_staged(self):
        """get_python_headers should support staged headers"""

        stage_dir = "stage_dir"

        # Fake out glob so it looks like the headers are in the staging area
        def _fake_glob(pattern):
            if pattern.startswith(stage_dir):
                return [os.path.join(stage_dir, "usr", "include", "pythontest")]
            return []

        self.mock_glob.side_effect = _fake_glob

        self.assertThat(
            _python_finder.get_python_headers("test", stage_dir=stage_dir),
            Equals(os.path.join(stage_dir, "usr", "include", "pythontest")),
        )

    def test_host(self):
        """get_python_headers should support the hosts's headers"""

        stage_dir = "stage_dir"

        # Fake out glob so it looks like the headers are installed on the host
        def _fake_glob(pattern):
            if pattern.startswith(os.sep):
                return [os.path.join(os.sep, "usr", "include", "pythontest")]
            return []

        self.mock_glob.side_effect = _fake_glob

        self.assertThat(
            _python_finder.get_python_headers("test", stage_dir=stage_dir),
            MatchesRegex(
                re.escape(os.path.join(os.sep, "usr", "include", "pythontest"))
            ),
        )

    def test_staged_and_host(self):
        """get_python_headers should prefer staged headers over host"""

        stage_dir = "stage_dir"

        # Fake out glob so it looks like the headers are in the staging area
        # AND on the host
        def _fake_glob(pattern):
            if pattern.startswith(stage_dir):
                return [os.path.join(stage_dir, "usr", "include", "pythontest")]
            elif pattern.startswith(os.sep):
                return [os.path.join(os.sep, "usr", "include", "pythontest")]
            return []

        self.mock_glob.side_effect = _fake_glob

        self.assertThat(
            _python_finder.get_python_headers("test", stage_dir=stage_dir),
            Equals(os.path.join(stage_dir, "usr", "include", "pythontest")),
        )

    def test_none(self):
        """get_python_headers should return an empty string if no headers"""

        stage_dir = "stage_dir"

        # Fake out glob so it looks like the headers aren't installed anywhere
        def _fake_glob(pattern):
            return []

        self.mock_glob.side_effect = _fake_glob

        self.assertFalse(_python_finder.get_python_headers("test", stage_dir=stage_dir))


class GetPythonHomeTestCase(PythonBaseTestCase):
    def test_staged(self):
        """get_python_home should support staged python"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create staged binary
        self._create_python_binary(stage_dir)

        python_home = _python_finder.get_python_home(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(python_home, Equals(os.path.join(stage_dir, "usr")))

    def test_in_part(self):
        """get_python_home should support in-part python"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create installed binary
        self._create_python_binary(install_dir)

        python_home = _python_finder.get_python_home(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(python_home, Equals(os.path.join(install_dir, "usr")))

    def test_staged_and_in_part(self):
        """get_python_home should prefer staged python over in-part"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create both staged and installed binaries
        self._create_python_binary(stage_dir)
        self._create_python_binary(install_dir)

        python_home = _python_finder.get_python_home(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )
        self.assertThat(python_home, Equals(os.path.join(stage_dir, "usr")))

    def test_missing_raises(self):
        """get_python_home should raise if no python can be found"""

        stage_dir = "stage_dir"
        install_dir = "install_dir"

        raised = self.assertRaises(
            errors.MissingPythonCommandError,
            _python_finder.get_python_home,
            "test",
            stage_dir=stage_dir,
            install_dir=install_dir,
        )
        self.assertThat(
            str(raised),
            Equals("Unable to find pythontest, searched: stage_dir:install_dir"),
        )
