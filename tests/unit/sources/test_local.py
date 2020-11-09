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

import copy
import os
import shutil
from unittest import mock

from testtools.matchers import DirExists, Equals, FileContains, FileExists, Not

from snapcraft.internal import common, errors, sources
from tests import unit


class TestLocal(unit.TestCase):
    @mock.patch("snapcraft.internal.sources._local.glob.glob")
    def test_pull_does_not_change_snapcraft_files_list(self, mock_glob):
        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1614913
        # Verify that SNAPCRAFT_FILES was not modified by the pull when there
        # are files to ignore.
        snapcraft_files_before_pull = copy.copy(common.SNAPCRAFT_FILES)
        mock_glob.return_value = ["a.snap", "b.snap", "c.snap"]

        local = sources.Local(".", "destination")
        local.pull()

        self.assertThat(snapcraft_files_before_pull, Equals(common.SNAPCRAFT_FILES))

    def test_pull_with_existing_empty_source_dir_creates_hardlinks(self):
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        os.mkdir("destination")

        local = sources.Local("src", "destination")
        local.pull()

        # Verify that the directories are not symlinks, but the file is a
        # hardlink.
        self.assertFalse(os.path.islink("destination"))
        self.assertFalse(os.path.islink(os.path.join("destination", "dir")))
        self.assertGreater(
            os.stat(os.path.join("destination", "dir", "file")).st_nlink, 1
        )

    def test_pull_with_existing_source_tree_creates_hardlinks(self):
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        os.mkdir("destination")
        open(os.path.join("destination", "existing-file"), "w").close()

        local = sources.Local("src", "destination")
        local.pull()

        # Verify that the directories are not symlinks, but the file is a
        # hardlink. Also verify that existing-file still exists.
        self.assertFalse(os.path.islink("destination"))
        self.assertFalse(os.path.islink(os.path.join("destination", "dir")))
        self.assertThat(os.path.join("destination", "existing-file"), FileExists())
        self.assertGreater(
            os.stat(os.path.join("destination", "dir", "file")).st_nlink, 1
        )

    def test_pull_with_existing_source_link_error(self):
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        # Note that this is a symlink now instead of a directory
        os.symlink("dummy", "destination")

        local = sources.Local("src", "destination")
        self.assertRaises(errors.SnapcraftEnvironmentError, local.pull)

    def test_pull_with_existing_source_file_error(self):
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        # Note that this is a file now instead of a directory
        open("destination", "w").close()

        local = sources.Local("src", "destination")
        self.assertRaises(errors.SnapcraftEnvironmentError, local.pull)

    def test_pulling_twice_with_existing_source_dir_recreates_hardlinks(self):
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        os.mkdir("destination")

        local = sources.Local("src", "destination")
        local.pull()
        local.pull()

        # Verify that the directories are not symlinks, but the file is a
        # hardlink.
        self.assertFalse(os.path.islink("destination"))
        self.assertFalse(os.path.islink(os.path.join("destination", "dir")))
        self.assertGreater(
            os.stat(os.path.join("destination", "dir", "file")).st_nlink, 1
        )

    def test_pull_ignores_snapcraft_specific_data(self):
        # Make the snapcraft-specific directories
        os.makedirs(os.path.join("src", "parts"))
        os.makedirs(os.path.join("src", "stage"))
        os.makedirs(os.path.join("src", "prime"))
        os.makedirs(os.path.join("src", ".snapcraft"))
        os.makedirs(os.path.join("src", "snap"))

        # Make the snapcraft.yaml (and hidden one) and a built snap
        open(os.path.join("src", "snapcraft.yaml"), "w").close()
        open(os.path.join("src", ".snapcraft.yaml"), "w").close()
        open(os.path.join("src", "foo.snap"), "w").close()

        # Make the global state cache
        open(os.path.join("src", ".snapcraft", "state"), "w").close()

        # Now make some real files
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()

        os.mkdir("destination")

        local = sources.Local("src", "destination")
        local.pull()

        # Verify that the snapcraft-specific stuff got filtered out
        self.assertThat(os.path.join("destination", "parts"), Not(DirExists()))
        self.assertThat(os.path.join("destination", "stage"), Not(DirExists()))
        self.assertThat(os.path.join("destination", "prime"), Not(DirExists()))

        self.assertThat(os.path.join("destination", "snap"), DirExists())
        self.assertThat(os.path.join("destination", ".snapcraft.yaml"), FileExists())
        self.assertThat(os.path.join("destination", "snapcraft.yaml"), FileExists())

        self.assertThat(os.path.join("destination", "foo.snap"), Not(FileExists()))

        # Verify that the real stuff made it in.
        self.assertFalse(os.path.islink("destination"))
        self.assertFalse(os.path.islink(os.path.join("destination", "dir")))
        self.assertGreater(
            os.stat(os.path.join("destination", "dir", "file")).st_nlink, 1
        )

    def test_pull_keeps_symlinks(self):
        # Create a source containing a directory, a file and symlinks to both.
        os.makedirs(os.path.join("src", "dir"))
        open(os.path.join("src", "dir", "file"), "w").close()
        os.symlink("dir", os.path.join("src", "dir_symlink"))
        os.symlink("file", os.path.join("src", "dir", "file_symlink"))

        local = sources.Local("src", "destination")
        local.pull()

        # Verify that both the file and the directory symlinks were kept.
        self.expectThat(os.path.join("destination", "dir"), DirExists())
        self.expectThat(
            os.path.join("destination", "dir_symlink"), unit.LinkExists("dir")
        )
        self.expectThat(os.path.join("destination", "dir", "file"), FileExists())
        self.expectThat(
            os.path.join("destination", "dir", "file_symlink"), unit.LinkExists("file")
        )

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["local"] is sources.Local)


class TestLocalIgnores:
    """Verify that the snapcraft root dir does not get copied into itself."""

    scenarios = [
        (f, dict(snapcraft_dir=f)) for f in common.SNAPCRAFT_FILES if "." not in f
    ]

    def test_pull_with_source_the_parent_of_current_dir(
        self, tmp_work_path, snapcraft_dir
    ):
        os.makedirs("subdir")

        cwd = tmp_work_path.as_posix()
        os.chdir("subdir")
        source_dir = os.path.join(snapcraft_dir, "foo_src")
        local = sources.Local("..", source_dir)
        local.pull()
        os.chdir(cwd)

        assert not os.path.exists(
            os.path.join("subdir", source_dir, "subdir", snapcraft_dir)
        )
        assert "subdir" in os.listdir(os.path.join("subdir", source_dir))

    def test_pull_with_source_a_parent_of_current_dir(
        self, tmp_work_path, snapcraft_dir
    ):
        subdir = os.path.join("subdir", "subsubdir", "subsubsubdir")
        os.makedirs(subdir)

        cwd = tmp_work_path.as_posix()
        os.chdir(subdir)
        source = "../" * (subdir.count(os.sep) + 1)
        source_dir = os.path.join(snapcraft_dir, "foo_src")
        local = sources.Local(source, source_dir)
        local.pull()
        os.chdir(cwd)
        assert not os.path.exists(
            os.path.join(subdir, source_dir, subdir, snapcraft_dir)
        )
        assert os.path.basename(subdir) in os.listdir(
            os.path.join(subdir, source_dir, os.path.dirname(subdir))
        )


class TestLocalUpdate(unit.TestCase):
    """Verify that the local source can detect changes and update."""

    def test_file_modified(self):
        source = "source"
        destination = "destination"
        os.mkdir(source)
        os.mkdir(destination)

        with open(os.path.join(source, "file"), "w") as f:
            f.write("1")

        # Now make a reference file with a timestamp later than the file was
        # created. We'll ensure this by setting it ourselves
        shutil.copy2(os.path.join(source, "file"), "reference")
        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime("reference", (access_time, modify_time + 1))

        local = sources.Local(source, destination)
        local.pull()
        self.assertFalse(
            local.check("reference"), "Expected no updates to be available"
        )
        self.assertThat(os.path.join(destination, "file"), FileContains("1"))

        # Now update the file in source, and make sure it has a timestamp
        # later than our reference (this whole test happens too fast)
        with open(os.path.join(source, "file"), "w") as f:
            f.write("2")

        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime(os.path.join(source, "file"), (access_time, modify_time + 1))

        self.assertTrue(local.check("reference"), "Expected update to be available")

        local.update()
        self.assertThat(os.path.join(destination, "file"), FileContains("2"))

    def test_file_added(self):
        source = "source"
        destination = "destination"
        os.mkdir(source)
        os.mkdir(destination)

        with open(os.path.join(source, "file1"), "w") as f:
            f.write("1")

        # Now make a reference file with a timestamp later than the file was
        # created. We'll ensure this by setting it ourselves
        shutil.copy2(os.path.join(source, "file1"), "reference")
        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime("reference", (access_time, modify_time + 1))

        local = sources.Local(source, destination)
        local.pull()
        self.assertFalse(
            local.check("reference"), "Expected no updates to be available"
        )
        self.assertThat(os.path.join(destination, "file1"), FileExists())

        # Now add a new file, and make sure it has a timestamp
        # later than our reference (this whole test happens too fast)
        with open(os.path.join(source, "file2"), "w") as f:
            f.write("2")

        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime(os.path.join(source, "file2"), (access_time, modify_time + 1))

        self.assertTrue(local.check("reference"), "Expected update to be available")

        local.update()
        self.assertThat(os.path.join(destination, "file2"), FileExists())

    def test_directory_modified(self):
        source = "source"
        source_dir = os.path.join(source, "dir")
        destination = "destination"
        os.makedirs(source_dir)
        os.mkdir(destination)

        with open(os.path.join(source_dir, "file1"), "w") as f:
            f.write("1")

        # Now make a reference file with a timestamp later than the file was
        # created. We'll ensure this by setting it ourselves
        shutil.copy2(os.path.join(source_dir, "file1"), "reference")
        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime("reference", (access_time, modify_time + 1))

        local = sources.Local(source, destination)
        local.pull()
        self.assertFalse(
            local.check("reference"), "Expected no updates to be available"
        )
        self.assertThat(os.path.join(destination, "dir", "file1"), FileExists())

        # Now add a new file to the directory, and make sure it has a timestamp
        # later than our reference (this whole test happens too fast)
        with open(os.path.join(source_dir, "file2"), "w") as f:
            f.write("2")

        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime(os.path.join(source_dir, "file2"), (access_time, modify_time + 1))

        self.assertTrue(local.check("reference"), "Expected update to be available")

        local.update()
        self.assertThat(os.path.join(destination, "dir", "file2"), FileExists())


class TestLocalUpdateSnapcraftYaml:

    scenarios = [
        ("snapcraft.yaml", dict(snapcraft_file="snapcraft.yaml")),
        (".snapcraft.yaml", dict(snapcraft_file=".snapcraft.yaml")),
        (
            "snap/snapcraft.yaml",
            dict(snapcraft_file=os.path.join("snap", "snapcraft.yaml")),
        ),
        ("snap/<content>", dict(snapcraft_file=os.path.join("snap", "test-file"))),
    ]

    def test_snapcraft_yaml_modification_ignored(self, tmp_work_path, snapcraft_file):
        source = "source"
        destination = "destination"
        snapcraft_source_path = os.path.join(source, snapcraft_file)
        snapcraft_destination_path = os.path.join(destination, snapcraft_file)
        os.makedirs(os.path.dirname(snapcraft_source_path))
        os.mkdir(destination)

        with open(snapcraft_source_path, "w") as f:
            f.write("1")

        # Now make a reference file with a timestamp later than the file was
        # created. We'll ensure this by setting it ourselves
        shutil.copy2(snapcraft_source_path, "reference")
        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime("reference", (access_time, modify_time + 1))

        local = sources.Local(source, destination)
        local.pull()
        assert not local.check("reference")
        assert os.path.exists(snapcraft_destination_path)

        # Now add a new file to the directory, and make sure it has a timestamp
        # later than our reference (this whole test happens too fast)
        with open(snapcraft_source_path, "w") as f:
            f.write("2")

        access_time = os.stat("reference").st_atime
        modify_time = os.stat("reference").st_mtime
        os.utime(snapcraft_source_path, (access_time, modify_time + 1))

        assert not local.check("reference")
