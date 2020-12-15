# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
import tarfile
from collections import OrderedDict
from pathlib import Path

from testtools.matchers import Equals

from snapcraft import yaml_utils
from snapcraft.internal.remote_build import WorkTree
from snapcraft.project import Project
from tests import fixture_setup, unit

from . import TestDir


class WorkTreeTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self._source = self.useFixture(TestDir())
        self._source.create_file("foo")
        self._source.create_file("bar")
        self._source.create_dir("dir")
        self._source.create_file("dir", "baz")
        self._source.create_dir("empty_dir")

        self._dest = self.useFixture(TestDir())

        self._snapcraft_yaml = fixture_setup.SnapcraftYaml(self._source.path)
        self._snapcraft_yaml.update_part(
            "my-part", {"plugin": "nil", "source": self._source.path}
        )
        self.useFixture(self._snapcraft_yaml)
        self.load_project_and_worktree()

    def load_project_and_worktree(self):
        """(Re)load project and worktree."""
        self._project = Project(
            snapcraft_yaml_file_path=self._snapcraft_yaml.snapcraft_yaml_file_path
        )
        self._project._project_dir = self._source.path
        self._wt = WorkTree(self._dest.path, self._project)

    def archive_path(self, part_name, source_selector=None):
        if source_selector:
            return os.path.join(
                self._dest.path,
                "repo",
                "sources",
                part_name,
                source_selector,
                part_name + ".tar.gz",
            )
        return os.path.join(
            self._dest.path, "repo", "sources", part_name, part_name + ".tar.gz"
        )

    def tarball_file_list(self, part_name, source_selector):
        archive_path = self.archive_path(part_name, source_selector)
        file_list = []
        with tarfile.open(archive_path, "r") as t:
            for m in t.getmembers():
                file_list.append(os.path.relpath(m.name, "./"))
        return file_list

    def tarball_file_contains(self, part_name, source_selector, *paths):
        file_list = self.tarball_file_list(part_name, source_selector)
        return os.path.join(*paths) in file_list

    def test_worktree_creation(self):
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache"))
        self.assertTrue(self._dest.exists("cache", "my-part"))
        self.assertTrue(self._dest.exists("repo"))
        self.assertTrue(self._dest.exists("repo", "sources"))
        self.assertTrue(self._dest.exists("repo", "sources", "my-part"))
        self.assertTrue(
            self._dest.exists("repo", "sources", "my-part", "my-part.tar.gz")
        )
        self.assertTrue(self._dest.exists("repo", "snap"))
        self.assertTrue(self._dest.exists("repo", "snap", "snapcraft.yaml"))

    def test_worktree_cache_new_file(self):
        self._wt.prepare_repository()
        self._source.create_file("new_file")
        self.assertFalse(self._dest.exists("cache", "my-part", "new_file"))
        self.assertFalse(self.tarball_file_contains("my-part", None, "new_file"))
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache", "my-part", "new_file"))
        self.assertTrue(self.tarball_file_contains("my-part", None, "new_file"))

    def test_worktree_cache_remove_file(self):
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache", "my-part", "foo"))
        self.assertTrue(self.tarball_file_contains("my-part", None, "foo"))
        self._source.unlink("foo")
        self._wt.prepare_repository()
        self.assertFalse(self._dest.exists("cache", "my-part", "foo"))
        self.assertFalse(self.tarball_file_contains("my-part", None, "foo"))

    def test_worktree_cache_remove_empty_dir(self):
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache", "my-part", "empty_dir"))
        self.assertTrue(self.tarball_file_contains("my-part", None, "empty_dir"))
        self._source.rmdir("empty_dir")
        self._wt.prepare_repository()
        self.assertFalse(self._dest.exists("cache", "my-part", "empty_dir"))
        self.assertFalse(self.tarball_file_contains("my-part", None, "empty_dir"))

    def test_worktree_cache_remove_dir(self):
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache", "my-part", "dir"))
        self.assertTrue(self.tarball_file_contains("my-part", None, "dir"))
        self._source.unlink("dir", "baz")
        self._source.rmdir("dir")
        self._wt.prepare_repository()
        self.assertFalse(self._dest.exists("cache", "my-part", "dir"))
        self.assertFalse(self.tarball_file_contains("my-part", None, "dir"))

    def test_assets_yaml(self):
        self._source.create_dir("snap", "gui")
        self._source.create_file("snap", "gui", "test.desktop")
        self._source.create_file("snap", "gui", "test.png")
        self.assertFalse(self._dest.exists("repo", "snap", "gui", "test.desktop"))
        self.assertFalse(self._dest.exists("repo", "snap", "gui", "test.png"))
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("repo", "snap", "gui", "test.desktop"))
        self.assertTrue(self._dest.exists("repo", "snap", "gui", "test.png"))

    def test_deprecated_local_plugins(self):
        self._source.create_dir("snap", "plugins")
        self._source.create_file("snap", "plugins", "plugin.py")
        self.assertFalse(self._dest.exists("repo", "snap", "plugins", "plugin.py"))
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("repo", "snap", "plugins", "plugin.py"))

    def test_source_selectors(self):
        self._snapcraft_yaml.update_part(
            "src-selectors",
            {
                "plugin": "nil",
                "source": [
                    {"on i386": self._source.path},
                    {"on arm64": self._source.path},
                ],
            },
        )
        self.load_project_and_worktree()
        self._wt.prepare_repository()
        self.assertTrue(self._dest.exists("cache"))
        self.assertTrue(self._dest.exists("cache", "src-selectors"))
        self.assertTrue(self._dest.exists("cache", "src-selectors", "on i386"))
        self.assertTrue(self._dest.exists("cache", "src-selectors", "on arm64"))
        self.assertTrue(self._dest.exists("repo"))
        self.assertTrue(self._dest.exists("repo", "sources"))
        self.assertTrue(self._dest.exists("repo", "sources", "my-part"))
        self.assertTrue(
            self._dest.exists("repo", "sources", "my-part", "my-part.tar.gz")
        )
        self.assertTrue(
            self._dest.exists(
                "repo", "sources", "src-selectors", "on i386", "src-selectors.tar.gz"
            )
        )
        self.assertTrue(
            self._dest.exists(
                "repo", "sources", "src-selectors", "on arm64", "src-selectors.tar.gz"
            )
        )
        self.assertFalse(
            self._dest.exists(
                "repo", "sources", "src-selectors", "src-selectors.tar.gz"
            )
        )
        self.assertTrue(self._dest.exists("repo", "snap"))
        self.assertTrue(self._dest.exists("repo", "snap", "snapcraft.yaml"))

    def test_stripped_source_keys(self):
        self._snapcraft_yaml.update_part(
            "my-part",
            {
                "plugin": "nil",
                "source": self._source.path,
                "source-checksum": "strip-me",
                "source-branch": "strip-me",
                "source-commit": "strip-me",
                "source-depth": "strip-me",
                "source-subdir": "test-sub-dir",
                "source-tag": "strip-me",
                "source-type": "local",
            },
        )

        self.load_project_and_worktree()
        self._wt.prepare_repository()

        config = yaml_utils.load_yaml_file(
            str(Path(self._dest.path, "repo", "snap", "snapcraft.yaml"))
        )

        self.assertThat(
            config["parts"]["my-part"],
            Equals(
                OrderedDict(
                    [
                        ("plugin", "nil"),
                        ("source", "sources/my-part/my-part.tar.gz"),
                        ("source-subdir", "test-sub-dir"),
                        ("source-type", "tar"),
                    ]
                )
            ),
        )
