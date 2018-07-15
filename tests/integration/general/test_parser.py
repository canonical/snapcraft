# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import fixtures
import subprocess


import testscenarios
from testtools.matchers import FileExists, Not

from tests import fixture_setup, integration


class ParserTestCase(integration.TestCase):
    """Test bin/snapcraft-parser"""

    def setUp(self):
        super().setUp()
        if os.getenv("SNAPCRAFT_FROM_SNAP", False):
            self.skipTest("The snapcraft-parser is not provided by the snap")
        self.useFixture(fixtures.EnvironmentVariable("TMPDIR", self.path))

    def call_parser(self, wiki_path, expect_valid, expect_output=True):
        part_file = os.path.join(self.path, "parts.yaml")
        args = ["--index", wiki_path, "--output", part_file]

        if expect_valid:
            self.run_snapcraft_parser(args)
        else:
            self.assertRaises(
                subprocess.CalledProcessError, self.run_snapcraft_parser, args
            )

        if expect_output:
            self.assertThat(part_file, FileExists())
        else:
            self.assertThat(part_file, Not(FileExists()))


class TestParser(ParserTestCase):
    """Test bin/snapcraft-parser"""

    def test_parser_basic(self):
        """Test snapcraft-parser basic usage"""
        fixture = fixture_setup.FakePartsWiki()
        self.useFixture(fixture)

        self.call_parser(fixture.fake_parts_wiki_fixture.url, expect_valid=True)


class TestParserWikis(testscenarios.WithScenarios, ParserTestCase):
    """Test bin/snapcraft-parser"""

    def setUp(self):
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable(
                "XDG_CACHE_HOME", os.path.join(self.path, ".cache")
            )
        )

    def _read_file(self, filename):
        content = ""
        with open(filename) as fp:
            content = fp.read()

        return content

    def _setup_wiki_file(self, filename, origin, commit=None):
        content = self._read_file(filename)
        content = content.replace("$ORIGIN", origin)
        if commit:
            content = content.replace("$COMMIT", commit)
        with open("wiki", "w") as fp:
            fp.write(content)

    def _setup_origin(self, snapcraft_files, repo_dir, base_dir):
        os.makedirs(repo_dir)
        previous_dir = os.getcwd()
        os.chdir(repo_dir)

        subprocess.check_call(
            ["git", "init", "."], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        subprocess.check_output(["git", "config", "user.name", "Test User"])
        subprocess.check_output(
            ["git", "config", "user.email", "<test.user@example.com"]
        )

        for snapcraft_file in snapcraft_files:
            snapcraft_content_path = os.path.join(base_dir, snapcraft_file["path"])
            snapcraft_content = self._read_file(snapcraft_content_path)
            snapcraft_file = snapcraft_file["snapcraft_file"]
            # make subdirectories if needed.
            if os.path.dirname(snapcraft_file):
                os.makedirs(os.path.dirname(snapcraft_file))
            with open(snapcraft_file, "w") as fp:
                fp.write(snapcraft_content)
            subprocess.check_call(
                ["git", "add", snapcraft_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        subprocess.check_call(
            ["git", "commit", "-am", "commit"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        subprocess.check_call(["git", "branch", "feature-branch"])
        commit = (
            subprocess.check_output(["git", "log"])
            .split()[0]
            .decode("utf-8")
            .replace("commit: ", "")
        )

        os.chdir(previous_dir)
        return commit

    scenarios = [
        (
            "Hidden .snapcraft.yaml file",
            {
                "wiki_file": "hidden_parts_wiki",
                "expect_valid": True,
                "expect_output": True,
                "snapcraft_files": [
                    {
                        "path": "hidden_snapcraft_yaml",
                        "snapcraft_file": ".snapcraft.yaml",
                    }
                ],
            },
        ),
        (
            "Both snapcraft.yaml and .snapcraft.yaml files",
            {
                "wiki_file": "both_parts_wiki",
                "expect_valid": False,
                "expect_output": True,
                "snapcraft_files": [
                    {
                        "path": "hidden_snapcraft_yaml",
                        "snapcraft_file": ".snapcraft.yaml",
                    },
                    {
                        "path": "hidden_snapcraft_yaml",
                        "snapcraft_file": "snapcraft.yaml",
                    },
                ],
            },
        ),
        (
            "snap/snapcraft.yaml",
            {
                "wiki_file": "snap.snapcraft.yaml",
                "expect_valid": True,
                "expect_output": True,
                "snapcraft_files": [
                    {
                        "path": "hidden_snapcraft_yaml",
                        "snapcraft_file": "snap/snapcraft.yaml",
                    }
                ],
            },
        ),
        (
            "Missing .snapcraft.yaml file",
            {
                "wiki_file": "missing_parts_wiki",
                "expect_valid": False,
                "expect_output": True,
                "snapcraft_files": [
                    {
                        "path": "hidden_snapcraft_yaml",
                        "snapcraft_file": "missing_snapcraft.yaml",
                    }
                ],
            },
        ),
        (
            "Origin type, branch and commit options",
            {
                "wiki_file": "origin_options_wiki",
                "expect_valid": True,
                "expect_output": True,
                "snapcraft_files": [
                    {
                        "path": "origin_options_snapcraft_yaml",
                        "snapcraft_file": "snapcraft.yaml",
                    }
                ],
            },
        ),
        (
            "Origin type, branch and commit options (wrong values)",
            {
                "wiki_file": "wrong_origin_options_wiki",
                "expect_valid": False,
                "expect_output": False,
                "snapcraft_files": [
                    {
                        "path": "wrong_origin_options_snapcraft_yaml",
                        "snapcraft_file": "snapcraft.yaml",
                    }
                ],
            },
        ),
    ]

    def test_local_wiki(self):
        repo_dir = "repo"
        base_dir = os.path.join(os.path.dirname(integration.__file__), "wiki")
        wiki_file = os.path.join(base_dir, self.wiki_file)

        commit = self._setup_origin(self.snapcraft_files, repo_dir, base_dir)
        self._setup_wiki_file(wiki_file, repo_dir, commit)
        self.call_parser("wiki", self.expect_valid, self.expect_output)
