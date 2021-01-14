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
import re
import stat
import tempfile
from textwrap import dedent
from unittest.mock import Mock, patch

import fixtures
import pytest
from testtools.matchers import Contains, Equals, FileExists, Not

import snapcraft
from snapcraft.internal import (
    common,
    errors,
    lifecycle,
    pluginhandler,
    project_loader,
    repo,
    states,
    steps,
)
from snapcraft.internal.sources.errors import SnapcraftSourceUnhandledError
from snapcraft.project import Project
from tests import fixture_setup, unit

from . import mocks


class PluginTestCase(unit.TestCase):
    def test_build_with_subdir_copies_sourcedir(self):
        handler = self.load_part("test-part", part_properties={"source-subdir": "src"})

        sourcedir = handler.part_source_dir
        source_subdir = handler.plugin.options.source_subdir

        subdir = os.path.join(sourcedir, source_subdir)
        os.makedirs(subdir)
        open(os.path.join(sourcedir, "file1"), "w").close()
        open(os.path.join(subdir, "file2"), "w").close()

        self.assertThat(
            handler.part_build_work_dir,
            Equals(os.path.join(handler.plugin.build_basedir, source_subdir)),
        )

        handler.build()

        self.assertThat(os.path.join(handler.part_build_dir, "file1"), FileExists())
        self.assertThat(
            os.path.join(handler.part_build_dir, source_subdir, "file2"), FileExists()
        )

    def test_build_with_missing_metadata_file(self):
        handler = self.load_part(
            "test-part", part_properties={"parse-info": ["missing-file"]}
        )
        handler.makedirs()

        raised = self.assertRaises(errors.MissingMetadataFileError, handler.build)
        self.assertThat(raised.path, Equals("missing-file"))

    def test_build_without_subdir_copies_sourcedir(self):
        handler = self.load_part("test-part")

        os.makedirs(handler.part_source_dir)
        open(os.path.join(handler.part_source_dir, "file"), "w").close()

        self.assertThat(handler.part_build_dir, Equals(handler.plugin.build_basedir))

        handler.build()

        self.assertTrue(
            os.path.exists(os.path.join(handler.plugin.build_basedir, "file"))
        )

    @patch("os.path.isdir", return_value=False)
    def test_local_non_dir_source_path_must_raise_exception(self, mock_isdir):
        self.assertRaises(
            SnapcraftSourceUnhandledError,
            self.load_part,
            "test-part",
            part_properties={"source": "file"},
        )

        mock_isdir.assert_any_call("file")

    def test_fileset_include_excludes(self):
        stage_set = [
            "-etc",
            "opt/something",
            "-usr/lib/*.a",
            "usr/bin",
            r"\-everything",
            r"\\a",
        ]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertThat(
            include, Equals(["opt/something", "usr/bin", "-everything", r"\a"])
        )
        self.assertThat(exclude, Equals(["etc", "usr/lib/*.a"]))

    @patch.object(snapcraft.plugins.v1.nil.NilPlugin, "snap_fileset")
    def test_migratable_fileset_for_no_options_modification(self, mock_snap_fileset):
        """Making sure migratable_fileset_for() doesn't modify options"""

        mock_snap_fileset.return_value = ["baz"]

        handler = self.load_part("test-part")
        handler.plugin.options.snap = ["foo"]
        handler.plugin.options.stage = ["bar"]
        expected_options = copy.deepcopy(handler.plugin.options)

        handler.migratable_fileset_for(steps.STAGE)
        self.assertThat(
            vars(handler.plugin.options),
            Equals(vars(expected_options)),
            "Expected options to be unmodified",
        )

        handler.migratable_fileset_for(steps.BUILD)
        self.assertThat(
            vars(handler.plugin.options),
            Equals(vars(expected_options)),
            "Expected options to be unmodified",
        )

    def test_fileset_only_includes(self):
        stage_set = ["opt/something", "usr/bin"]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertThat(include, Equals(["opt/something", "usr/bin"]))
        self.assertThat(exclude, Equals([]))

    def test_fileset_only_excludes(self):
        stage_set = ["-etc", "-usr/lib/*.a"]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertThat(include, Equals(["*"]))
        self.assertThat(exclude, Equals(["etc", "usr/lib/*.a"]))

    def test_migrate_snap_files_already_exists(self):
        os.makedirs("install")
        os.makedirs("stage")

        # Place the already-staged file
        with open("stage/foo", "w") as f:
            f.write("staged")

        # Place the to-be-staged file with the same name
        with open("install/foo", "w") as f:
            f.write("installed")

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the staged file is the one that was staged last
        with open("stage/foo", "r") as f:
            self.assertThat(
                f.read(),
                Equals("installed"),
                "Expected staging to allow overwriting of already-staged files",
            )

    def test_migrate_files_supports_no_follow_symlinks(self):
        os.makedirs("install")
        os.makedirs("stage")

        with open(os.path.join("install", "foo"), "w") as f:
            f.write("installed")

        os.symlink("foo", os.path.join("install", "bar"))

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=False
        )

        # Verify that the symlink was preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'bar' to still be a symlink.",
        )
        self.assertThat(
            os.readlink(os.path.join("stage", "bar")),
            Equals("foo"),
            "Expected migrated 'bar' to point to 'foo'",
        )

    def test_migrate_files_preserves_symlink_file(self):
        os.makedirs("install")
        os.makedirs("stage")

        with open(os.path.join("install", "foo"), "w") as f:
            f.write("installed")

        os.symlink("foo", os.path.join("install", "bar"))

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'sym-a' to be a symlink.",
        )

    def test_migrate_files_no_follow_symlinks(self):
        os.makedirs("install/usr/bin")
        os.makedirs("stage")

        with open(os.path.join("install", "usr", "bin", "foo"), "w") as f:
            f.write("installed")

        os.symlink("usr/bin", os.path.join("install", "bin"))

        files, dirs = pluginhandler._migratable_filesets(["-usr"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        assert files == {"bin"}
        assert dirs == set()

        self.assertTrue(
            os.path.islink(os.path.join("stage", "bin")),
            "Expected migrated 'bin' to be a symlink.",
        )

    def test_migrate_files_preserves_symlink_nested_file(self):
        os.makedirs(os.path.join("install", "a"))
        os.makedirs("stage")

        with open(os.path.join("install", "a", "foo"), "w") as f:
            f.write("installed")

        os.symlink(os.path.join("a", "foo"), os.path.join("install", "bar"))
        os.symlink(os.path.join("foo"), os.path.join("install", "a", "bar"))

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'sym-a' to be a symlink.",
        )

        self.assertTrue(
            os.path.islink(os.path.join("stage", "a", "bar")),
            "Expected migrated 'sym-a' to be a symlink.",
        )

    def test_migrate_files_preserves_symlink_empty_dir(self):
        os.makedirs(os.path.join("install", "foo"))
        os.makedirs("stage")

        os.symlink("foo", os.path.join("install", "bar"))

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'sym-a' to be a symlink.",
        )

    def test_migrate_files_preserves_symlink_nonempty_dir(self):
        os.makedirs(os.path.join("install", "foo"))
        os.makedirs("stage")

        os.symlink("foo", os.path.join("install", "bar"))

        with open(os.path.join("install", "foo", "xfile"), "w") as f:
            f.write("installed")

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'sym-a' to be a symlink.",
        )

    def test_migrate_files_preserves_symlink_nested_dir(self):
        os.makedirs(os.path.join("install", "a", "b"))
        os.makedirs("stage")

        os.symlink(os.path.join("a", "b"), os.path.join("install", "bar"))
        os.symlink(os.path.join("b"), os.path.join("install", "a", "bar"))

        with open(os.path.join("install", "a", "b", "xfile"), "w") as f:
            f.write("installed")

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(files, dirs, "install", "stage")

        # Verify that the symlinks were preserved
        self.assertTrue(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'bar' to be a symlink.",
        )

        self.assertTrue(
            os.path.islink(os.path.join("stage", "a", "bar")),
            "Expected migrated 'a/bar' to be a symlink.",
        )

    def test_migrate_files_supports_follow_symlinks(self):
        os.makedirs("install")
        os.makedirs("stage")

        with open(os.path.join("install", "foo"), "w") as f:
            f.write("installed")

        os.symlink("foo", os.path.join("install", "bar"))

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        # Verify that the symlink was preserved
        self.assertFalse(
            os.path.islink(os.path.join("stage", "bar")),
            "Expected migrated 'bar' to no longer be a symlink.",
        )
        with open(os.path.join("stage", "bar"), "r") as f:
            self.assertThat(
                f.read(),
                Equals("installed"),
                "Expected migrated 'bar' to be a copy of 'foo'",
            )

    def test_migrate_files_preserves_file_mode(self):
        os.makedirs("install")
        os.makedirs("stage")

        foo = os.path.join("install", "foo")

        with open(foo, "w") as f:
            f.write("installed")

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        os.chmod(foo, new_mode)
        self.assertNotEqual(mode, new_mode)

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        self.assertThat(
            new_mode,
            Equals(stat.S_IMODE(os.stat(os.path.join("stage", "foo")).st_mode)),
        )

    def test_migrate_files_preserves_file_mode_chown_permissions(self):
        os.makedirs("install")
        os.makedirs("stage")

        foo = os.path.join("install", "foo")

        with open(foo, "w") as f:
            f.write("installed")

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        os.chmod(foo, new_mode)
        self.assertNotEqual(mode, new_mode)

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        self.assertThat(
            new_mode,
            Equals(stat.S_IMODE(os.stat(os.path.join("stage", "foo")).st_mode)),
        )

    def test_migrate_files_preserves_directory_mode(self):
        os.makedirs("install/foo")
        os.makedirs("stage")

        foo = os.path.join("install", "foo", "bar")

        with open(foo, "w") as f:
            f.write("installed")

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        self.assertNotEqual(mode, new_mode)
        os.chmod(os.path.dirname(foo), new_mode)
        os.chmod(foo, new_mode)

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        self.assertThat(
            new_mode,
            Equals(stat.S_IMODE(os.stat(os.path.join("stage", "foo")).st_mode)),
        )
        self.assertThat(
            new_mode,
            Equals(stat.S_IMODE(os.stat(os.path.join("stage", "foo", "bar")).st_mode)),
        )

    def test_filesets_includes_without_relative_paths(self):
        raised = self.assertRaises(
            errors.PluginError, pluginhandler._get_file_list, ["rel", "/abs/include"]
        )

        self.assertThat(raised.message, Equals('path "/abs/include" must be relative'))

    def test_filesets_excludes_without_relative_paths(self):
        raised = self.assertRaises(
            errors.PluginError, pluginhandler._get_file_list, ["rel", "-/abs/exclude"]
        )

        self.assertThat(raised.message, Equals('path "/abs/exclude" must be relative'))

    @patch("snapcraft.internal.pluginhandler._organize_filesets")
    def test_build_organizes(self, mock_organize):
        handler = self.load_part("test-part")
        handler.build()
        mock_organize.assert_called_once_with(
            "test-part", {}, handler.part_install_dir, False
        )

    @patch("snapcraft.internal.pluginhandler._organize_filesets")
    def test_update_build_organizes_with_overwrite(self, mock_organize):
        class TestPlugin(snapcraft.BasePlugin):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.out_of_source_build = True

        self.useFixture(fixture_setup.FakePlugin("test-plugin", TestPlugin))

        handler = self.load_part("test-part", plugin_name="test-plugin")
        handler.makedirs()
        handler.update_build()
        mock_organize.assert_called_once_with(
            "test-part", {}, handler.part_install_dir, True
        )


class TestMigratePartFiles:

    scenarios = [
        ("nothing", {"fileset": ["-*"], "result": []}),
        (
            "all",
            {
                "fileset": ["*"],
                "result": [
                    "stage/1",
                    "stage/1/1a/1b",
                    "stage/1/1a",
                    "stage/1/a",
                    "stage/2",
                    "stage/2/2a",
                    "stage/3",
                    "stage/3/a",
                    "stage/a",
                    "stage/b",
                ],
            },
        ),
        (
            "no1",
            {
                "fileset": ["-1"],
                "result": [
                    "stage/2",
                    "stage/2/2a",
                    "stage/3",
                    "stage/3/a",
                    "stage/a",
                    "stage/b",
                ],
            },
        ),
        ("onlya", {"fileset": ["a"], "result": ["stage/a"]}),
        (
            "onlybase",
            {
                "fileset": ["*", "-*/*"],
                "result": ["stage/a", "stage/b", "stage/1", "stage/2", "stage/3"],
            },
        ),
        (
            "nostara",
            {
                "fileset": ["-*/a"],
                "result": [
                    "stage/1",
                    "stage/1/1a/1b",
                    "stage/1/1a",
                    "stage/2",
                    "stage/2/2a",
                    "stage/3",
                    "stage/a",
                    "stage/b",
                ],
            },
        ),
    ]

    def test_migrate_snap_files(self, tmp_path, fileset, result):
        srcdir = tmp_path / "install"

        (srcdir / "1/1a/1b").mkdir(parents=True)
        (srcdir / "2/2a").mkdir(parents=True)
        (srcdir / "3").mkdir(parents=True)

        (srcdir / "a").touch()
        (srcdir / "b").touch()
        (srcdir / "1/a").touch()
        (srcdir / "3/a").touch()

        dstdir = tmp_path / "stage"
        dstdir.mkdir(parents=True)

        files, dirs = pluginhandler._migratable_filesets(fileset, srcdir.as_posix())
        pluginhandler._migrate_files(files, dirs, srcdir.as_posix(), dstdir.as_posix())

        expected = []
        for item in result:
            expected.append(os.path.join(tmp_path, item))
        expected.sort()

        result = []
        for root, subdirs, files in os.walk(dstdir):
            for item in files:
                result.append(os.path.join(root, item))
            for item in subdirs:
                result.append(os.path.join(root, item))
        result.sort()

        assert result == expected


class MigratableFilesetsTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        os.makedirs("install/foo/bar/baz")
        open("install/1", "w").close()
        open("install/foo/2", "w").close()
        open("install/foo/bar/3", "w").close()
        open("install/foo/bar/baz/4", "w").close()

    def test_migratable_filesets_everything(self):
        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        self.assertThat(files, Equals({"1", "foo/2", "foo/bar/3", "foo/bar/baz/4"}))
        self.assertThat(dirs, Equals({"foo", "foo/bar", "foo/bar/baz"}))

    def test_migratable_filesets_foo(self):
        files, dirs = pluginhandler._migratable_filesets(["foo"], "install")
        self.assertThat(files, Equals({"foo/2", "foo/bar/3", "foo/bar/baz/4"}))
        self.assertThat(dirs, Equals({"foo", "foo/bar", "foo/bar/baz"}))

    def test_migratable_filesets_everything_in_foo(self):
        files, dirs = pluginhandler._migratable_filesets(["foo/*"], "install")
        self.assertThat(files, Equals({"foo/2", "foo/bar/3", "foo/bar/baz/4"}))
        self.assertThat(dirs, Equals({"foo", "foo/bar", "foo/bar/baz"}))

    def test_migratable_filesets_root_file(self):
        files, dirs = pluginhandler._migratable_filesets(["1"], "install")
        self.assertThat(files, Equals({"1"}))
        self.assertThat(dirs, Equals(set()))

    def test_migratable_filesets_single_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(["foo/2"], "install")
        self.assertThat(files, Equals({"foo/2"}))
        self.assertThat(dirs, Equals({"foo"}))

    def test_migratable_filesets_single_really_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(["foo/bar/2"], "install")
        self.assertThat(files, Equals({"foo/bar/2"}))
        self.assertThat(dirs, Equals({"foo", "foo/bar"}))

    def test_migratable_filesets_single_really_really_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(["foo/bar/baz/3"], "install")
        self.assertThat(files, Equals({"foo/bar/baz/3"}))
        self.assertThat(dirs, Equals({"foo", "foo/bar", "foo/bar/baz"}))


class TestOrganize:

    scenarios = [
        (
            "simple_file",
            dict(
                setup_dirs=[],
                setup_files=["foo"],
                organize_set={"foo": "bar"},
                expected=[(["bar"], "")],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "simple_dir_with_file",
            dict(
                setup_dirs=["foodir"],
                setup_files=[os.path.join("foodir", "foo")],
                organize_set={"foodir": "bardir"},
                expected=[(["bardir"], ""), (["foo"], "bardir")],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "organize_to_the_same_directory",
            dict(
                setup_dirs=["bardir", "foodir"],
                setup_files=[
                    os.path.join("foodir", "foo"),
                    os.path.join("bardir", "bar"),
                    "basefoo",
                ],
                organize_set={
                    "foodir": "bin",
                    "bardir": "bin",
                    "basefoo": "bin/basefoo",
                },
                expected=[(["bin"], ""), (["bar", "basefoo", "foo"], "bin")],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "leading_slash_in_value",
            dict(
                setup_dirs=[],
                setup_files=["foo"],
                organize_set={"foo": "/bar"},
                expected=[(["bar"], "")],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "overwrite_existing_file",
            dict(
                setup_dirs=[],
                setup_files=["foo", "bar"],
                organize_set={"foo": "bar"},
                expected=errors.SnapcraftOrganizeError,
                expected_message=r".*trying to organize file 'foo' to 'bar', but 'bar' already exists.*",
                expected_overwrite=[(["bar"], "")],
            ),
        ),
        (
            "*_for_files",
            dict(
                setup_dirs=[],
                setup_files=["foo.conf", "bar.conf"],
                organize_set={"*.conf": "dir/"},
                expected=[(["dir"], ""), (["bar.conf", "foo.conf"], "dir")],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "*_for_files_with_non_dir_dst",
            dict(
                setup_dirs=[],
                setup_files=["foo.conf", "bar.conf"],
                organize_set={"*.conf": "dir"},
                expected=errors.SnapcraftOrganizeError,
                expected_message=r".*multiple files to be organized into 'dir'.*",
                expected_overwrite=None,
            ),
        ),
        (
            "*_for_directories",
            dict(
                setup_dirs=["dir1", "dir2"],
                setup_files=[os.path.join("dir1", "foo"), os.path.join("dir2", "bar")],
                organize_set={"dir*": "dir/"},
                expected=[
                    (["dir"], ""),
                    (["dir1", "dir2"], "dir"),
                    (["foo"], os.path.join("dir", "dir1")),
                    (["bar"], os.path.join("dir", "dir2")),
                ],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "combined_*_with_file",
            dict(
                setup_dirs=["dir1", "dir2"],
                setup_files=[
                    os.path.join("dir1", "foo"),
                    os.path.join("dir1", "bar"),
                    os.path.join("dir2", "bar"),
                ],
                organize_set={"dir*": "dir/", "dir1/bar": "."},
                expected=[
                    (["bar", "dir"], ""),
                    (["dir1", "dir2"], "dir"),
                    (["foo"], os.path.join("dir", "dir1")),
                    (["bar"], os.path.join("dir", "dir2")),
                ],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
        (
            "*_into_dir",
            dict(
                setup_dirs=["dir"],
                setup_files=[os.path.join("dir", "foo"), os.path.join("dir", "bar")],
                organize_set={"dir/f*": "nested/dir/"},
                expected=[
                    (["dir", "nested"], ""),
                    (["bar"], "dir"),
                    (["dir"], "nested"),
                    (["foo"], os.path.join("nested", "dir")),
                ],
                expected_message=None,
                expected_overwrite=None,
            ),
        ),
    ]

    def _organize_and_assert(
        self,
        tmp_path,
        setup_dirs,
        setup_files,
        organize_set,
        expected,
        expected_message,
        expected_overwrite,
        overwrite,
    ):
        base_dir = tmp_path / "install"
        base_dir.mkdir(parents=True, exist_ok=True)

        for directory in setup_dirs:
            (base_dir / directory).mkdir(exist_ok=True)

        for file_entry in setup_files:
            (base_dir / file_entry).touch()

        if overwrite and expected_overwrite is not None:
            expected = expected_overwrite

        if isinstance(expected, type) and issubclass(expected, Exception):
            with pytest.raises(expected) as error:
                pluginhandler._organize_filesets(
                    "part-name", organize_set, base_dir, overwrite
                )
            assert re.match(expected_message, str(error)) is not None

        else:
            pluginhandler._organize_filesets(
                "part-name", organize_set, base_dir, overwrite
            )
            for expect in expected:
                dir_path = (base_dir / expect[1]).as_posix()
                dir_contents = os.listdir(dir_path)
                dir_contents.sort()
                assert dir_contents == expect[0]

    def test_organize(
        self,
        tmp_path,
        setup_dirs,
        setup_files,
        organize_set,
        expected,
        expected_message,
        expected_overwrite,
    ):
        self._organize_and_assert(
            tmp_path,
            setup_dirs,
            setup_files,
            organize_set,
            expected,
            expected_message,
            expected_overwrite,
            False,
        )

        # Verify that it can be organized again by overwriting
        self._organize_and_assert(
            tmp_path,
            setup_dirs,
            setup_files,
            organize_set,
            expected,
            expected_message,
            expected_overwrite,
            True,
        )


class RealStageTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        fake_install_build_packages = fixtures.MockPatch(
            "snapcraft.internal.lifecycle._runner._install_build_packages",
            return_value=list(),
        )
        self.useFixture(fake_install_build_packages)

        fake_install_build_snaps = fixtures.MockPatch(
            "snapcraft.internal.lifecycle._runner._install_build_snaps",
            return_value=list(),
        )
        self.useFixture(fake_install_build_snaps)

    def make_snapcraft_project(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: pc-file-test
            base: core18
            version: "1.0"
            summary: test pkg-config .pc
            description: when the .pc files reach stage the should be reprefixed
            confinement: strict
            grade: stable

            parts:
                stage-pc:
                    plugin: nil
        """
            )
        )
        project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        return project_loader.load_config(project)

    def test_pc_files_correctly_prefixed(self):
        pc_file = os.path.join("usr", "lib", "pkgconfig", "granite.pc")
        stage_pc_install = os.path.join(self.parts_dir, "stage-pc", "install", pc_file)
        stage_pc_stage = os.path.join(self.stage_dir, pc_file)

        # Run build
        project_config = self.make_snapcraft_project()
        lifecycle.execute(steps.BUILD, project_config)

        # Simulate a .pc file was installed
        os.makedirs(os.path.dirname(stage_pc_install))
        with open(stage_pc_install, "w") as f:
            f.write("prefix=/usr\n")
            f.write("exec_prefix=${prefix}\n")
            f.write("libdir=${prefix}/lib\n")
            f.write("includedir=${prefix}/include\n")
            f.write("\n")
            f.write("Name: granite\n")
            f.write("Description: elementary's Application Framework\n")
            f.write("Version: 0.4\n")
            f.write("Libs: -L${libdir} -lgranite\n")
            f.write("Cflags: -I${includedir}/granite\n")
            f.write("Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0\n")

        # Now we stage
        lifecycle.execute(steps.STAGE, project_config)

        with open(stage_pc_stage) as f:
            pc_file_content = f.read()
        expected_pc_file_content = """prefix={}/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib
includedir=${{prefix}}/include

Name: granite
Description: elementary's Application Framework
Version: 0.4
Libs: -L${{libdir}} -lgranite
Cflags: -I${{includedir}}/granite
Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
""".format(
            self.stage_dir
        )

        self.assertThat(pc_file_content, Equals(expected_pc_file_content))

    def test_pc_files_correctly_prefixed_when_installed(self):
        pc_file = os.path.join("usr", "lib", "pkgconfig", "granite.pc")
        install_path = os.path.join(self.parts_dir, "stage-pc", "install")
        stage_pc_install = os.path.join(install_path, pc_file)
        stage_pc_stage = os.path.join(self.stage_dir, pc_file)

        # Run build
        project_config = self.make_snapcraft_project()
        lifecycle.execute(steps.BUILD, project_config)

        # Simulate a .pc file was installed
        os.makedirs(os.path.dirname(stage_pc_install))
        with open(stage_pc_install, "w") as f:
            f.write("prefix={}/usr\n".format(install_path))
            f.write("exec_prefix=${prefix}\n")
            f.write("libdir=${prefix}/lib\n")
            f.write("includedir=${prefix}/include\n")
            f.write("\n")
            f.write("Name: granite\n")
            f.write("Description: elementary's Application Framework\n")
            f.write("Version: 0.4\n")
            f.write("Libs: -L${libdir} -lgranite\n")
            f.write("Cflags: -I${includedir}/granite\n")
            f.write("Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0\n")

        # Now we stage
        lifecycle.execute(steps.STAGE, project_config)

        with open(stage_pc_stage) as f:
            pc_file_content = f.read()
        expected_pc_file_content = """prefix={}/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib
includedir=${{prefix}}/include

Name: granite
Description: elementary's Application Framework
Version: 0.4
Libs: -L${{libdir}} -lgranite
Cflags: -I${{includedir}}/granite
Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
""".format(
            self.stage_dir
        )

        self.assertThat(pc_file_content, Equals(expected_pc_file_content))


class NextLastStepTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.handler = self.load_part("test_part")

    def test_pull(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))

    def test_build(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))

    def test_stage(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))

    def test_prime(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)


class IsDirtyTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.handler = self.load_part("test-part")
        self.handler.makedirs()

    def test_prime_is_dirty(self):
        self.handler.plugin.options.snap = ["foo"]
        self.handler._part_properties = {"prime": ["foo"]}
        self.handler.mark_done(
            steps.PRIME,
            states.PrimeState(set(), set(), set(), self.handler._part_properties),
        )
        self.assertFalse(
            self.handler.is_clean(steps.PRIME), "Prime step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PRIME), "Strip step was unexpectedly dirty"
        )

        # Change the `snap` keyword-- thereby making the prime step dirty.
        self.handler.plugin.options.snap = ["bar"]
        self.handler._part_properties = {"prime": ["bar"]}
        self.assertFalse(
            self.handler.is_clean(steps.PRIME), "Strip step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.PRIME), "Expected prime step to be dirty"
        )

    def test_prime_not_dirty_if_clean(self):
        self.assertTrue(
            self.handler.is_clean(steps.PRIME),
            "Expected vanilla handler to have clean prime step",
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PRIME),
            "Expected vanilla handler to not have a dirty prime step",
        )

    def test_stage_is_dirty(self):
        self.handler = self.load_part("test-part", part_properties={"stage": ["foo"]})

        self.handler.mark_stage_done(set(), set())
        self.assertFalse(
            self.handler.is_clean(steps.STAGE), "Stage step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.STAGE), "Stage step was unexpectedly dirty"
        )

        # Change the `stage` keyword-- thereby making the stage step dirty.
        self.handler = self.load_part("test-part", part_properties={"stage": ["bar"]})
        self.assertFalse(
            self.handler.is_clean(steps.STAGE), "Stage step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.STAGE), "Expected stage step to be dirty"
        )

    def test_stage_not_dirty_if_clean(self):
        self.assertTrue(
            self.handler.is_clean(steps.STAGE),
            "Expected vanilla handler to have clean stage step",
        )
        self.assertFalse(
            self.handler.is_dirty(steps.STAGE),
            "Expected vanilla handler to not have a dirty stage step",
        )

    def test_build_is_dirty_from_options(self):
        self.useFixture(fixture_setup.FakePlugin("test-plugin", mocks.TestPlugin))
        self.handler = self.load_part(
            "test-part", "test-plugin", {"test-property": "foo"}
        )
        self.handler.mark_build_done()
        self.assertFalse(
            self.handler.is_clean(steps.BUILD), "Build step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.BUILD), "Build step was unexpectedly dirty"
        )

        # Change `test-property`, thereby making the build step dirty.
        self.handler = self.load_part(
            "test-part", "test-plugin", {"test-property": "bar"}
        )
        self.assertFalse(
            self.handler.is_clean(steps.BUILD), "Build step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.BUILD), "Expected build step to be dirty"
        )

    @patch.object(snapcraft.BasePlugin, "enable_cross_compilation")
    def test_build_is_dirty_from_project(self, mock_enable_cross_compilation):
        project = Project(target_deb_arch="amd64")
        self.handler = self.load_part("test-part", project=project)
        self.handler.mark_build_done()
        self.assertFalse(
            self.handler.is_clean(steps.BUILD), "Build step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.BUILD), "Build step was unexpectedly dirty"
        )

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project = Project(target_deb_arch="armhf")
        self.handler = self.load_part("test-part", project=project)
        self.assertFalse(
            self.handler.is_clean(steps.BUILD), "Build step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.BUILD), "Expected build step to be dirty"
        )

    def test_build_not_dirty_if_clean(self):
        self.assertTrue(
            self.handler.is_clean(steps.BUILD),
            "Expected vanilla handler to have clean build step",
        )
        self.assertFalse(
            self.handler.is_dirty(steps.BUILD),
            "Expected vanilla handler to not have a dirty build step",
        )

    def test_pull_is_dirty_from_options(self):
        self.useFixture(fixture_setup.FakePlugin("test-plugin", mocks.TestPlugin))
        self.handler = self.load_part(
            "test-part", "test-plugin", {"test-property": "foo"}
        )
        self.handler.mark_pull_done()
        self.assertFalse(
            self.handler.is_clean(steps.PULL), "Pull step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PULL), "Pull step was unexpectedly dirty"
        )

        # Change `test-property`, thereby making the pull step dirty.
        self.handler = self.load_part(
            "test-part", "test-plugin", {"test-property": "bar"}
        )
        self.assertFalse(
            self.handler.is_clean(steps.PULL), "Pull step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.PULL), "Expected pull step to be dirty"
        )

    @patch.object(snapcraft.BasePlugin, "enable_cross_compilation")
    def test_pull_is_dirty_from_project(self, mock_enable_cross_compilation):
        project = Project(target_deb_arch="amd64")
        self.handler = self.load_part("test-part", project=project)
        self.handler.mark_pull_done()
        self.assertFalse(
            self.handler.is_clean(steps.PULL), "Pull step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PULL), "Pull step was unexpectedly dirty"
        )

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project = Project(target_deb_arch="armhf")
        self.handler = self.load_part("test-part", project=project)
        self.assertFalse(
            self.handler.is_clean(steps.PULL), "Pull step was unexpectedly clean"
        )
        self.assertTrue(
            self.handler.is_dirty(steps.PULL), "Expected pull step to be dirty"
        )

    def test_pull_not_dirty_if_clean(self):
        self.assertTrue(
            self.handler.is_clean(steps.PULL),
            "Expected vanilla handler to have clean pull step",
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PULL),
            "Expected vanilla handler to not have a dirty pull step",
        )


class IsOutdatedTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.handler = self.load_part("test-part", part_properties=dict(source="."))
        self.handler.makedirs()

    def set_modified_time_later(self, target, reference):
        access_time = os.stat(reference).st_atime
        modified_time = os.stat(reference).st_atime
        os.utime(target, (access_time, modified_time + 1))

    def test_prime_is_outdated(self):
        self.handler.mark_prime_done(set(), set(), set(), set())
        self.assertFalse(
            self.handler.is_outdated(steps.PRIME),
            "Prime step was unexpectedly outdated",
        )

        # Now mark the stage state as done, but ensure it has a later
        # timestamp, thereby making the prime step out of date.
        prime_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.PRIME
        )
        stage_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.STAGE
        )
        open(stage_state_path, "w").close()
        self.set_modified_time_later(stage_state_path, prime_state_path)

        self.assertTrue(
            self.handler.is_outdated(steps.PRIME), "Expected prime step to be outdated"
        )

    def test_stage_is_outdated(self):
        self.handler.mark_stage_done(set(), set())

        self.assertFalse(
            self.handler.is_outdated(steps.STAGE),
            "Stage step was unexpectedly outdated",
        )

        # Now mark the build state as done, but ensure it has a later
        # timestamp, thereby making the stage step out of date.
        stage_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.STAGE
        )
        build_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.BUILD
        )
        open(build_state_path, "w").close()
        self.set_modified_time_later(build_state_path, stage_state_path)

        self.assertTrue(
            self.handler.is_outdated(steps.STAGE), "Expected stage step to be outdated"
        )

    def test_build_is_outdated(self):
        self.handler.mark_build_done()

        self.assertFalse(
            self.handler.is_outdated(steps.BUILD),
            "Build step was unexpectedly outdated",
        )

        # Now mark the pull state as done, but ensure it has a later
        # timestamp, thereby making the build step out of date.
        build_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.BUILD
        )
        pull_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.PULL
        )
        open(pull_state_path, "w").close()
        self.set_modified_time_later(pull_state_path, build_state_path)

        self.assertTrue(
            self.handler.is_outdated(steps.BUILD), "Expected build step to be outdated"
        )

    def test_pull_is_outdated(self):
        self.handler.mark_pull_done()

        self.assertFalse(
            self.handler.is_outdated(steps.PULL), "Pull step was unexpectedly outdated"
        )

        # Now update the source on disk, ensuring it has a later timestamp,
        # thereby making the pull step out of date.
        open("new_file", "w").close()
        pull_state_path = states.get_step_state_file(
            self.handler.plugin.statedir, steps.PULL
        )
        self.set_modified_time_later("new_file", pull_state_path)

        self.assertTrue(
            self.handler.is_outdated(steps.PULL), "Expected pull step to be outdated"
        )


class CollisionTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = self.load_part("part1")
        part1.part_install_dir = tmpdir + "/install1"
        os.makedirs(part1.part_install_dir + "/a")
        open(part1.part_install_dir + "/a/1", mode="w").close()
        with open(part1.part_install_dir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part1.part_install_dir))
            f.write("Name: File\n")

        part2 = self.load_part("part2")
        part2.part_install_dir = tmpdir + "/install2"
        os.makedirs(part2.part_install_dir + "/a")
        with open(part2.part_install_dir + "/1", mode="w") as f:
            f.write("1")
        open(part2.part_install_dir + "/2", mode="w").close()
        with open(part2.part_install_dir + "/a/2", mode="w") as f:
            f.write("a/2")
        with open(part2.part_install_dir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part2.part_install_dir))
            f.write("Name: File\n")

        part3 = self.load_part("part3")
        part3.part_install_dir = tmpdir + "/install3"
        os.makedirs(part3.part_install_dir + "/a")
        os.makedirs(part3.part_install_dir + "/b")
        with open(part3.part_install_dir + "/1", mode="w") as f:
            f.write("2")
        with open(part2.part_install_dir + "/2", mode="w") as f:
            f.write("1")
        open(part3.part_install_dir + "/a/2", mode="w").close()

        part4 = self.load_part("part4")
        part4.part_install_dir = tmpdir + "/install4"
        os.makedirs(part4.part_install_dir)
        with open(part4.part_install_dir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part4.part_install_dir))
            f.write("Name: ConflictFile\n")

        # Create a new part with a symlink that collides with part1's
        # non-symlink.
        part5 = self.load_part("part5")
        part5.part_install_dir = os.path.join(tmpdir, "install5")
        os.makedirs(part5.part_install_dir)
        os.symlink("foo", os.path.join(part5.part_install_dir, "a"))

        # Create a new part with a symlink that points to a different place
        # than part5's symlink.
        part6 = self.load_part("part6")
        part6.part_install_dir = os.path.join(tmpdir, "install6")
        os.makedirs(part6.part_install_dir)
        os.symlink("bar", os.path.join(part6.part_install_dir, "a"))

        self.part1 = part1
        self.part2 = part2
        self.part3 = part3
        self.part4 = part4
        self.part5 = part5
        self.part6 = part6

    def test_no_collisions(self):
        """No exception is expected as there are no collisions."""
        pluginhandler.check_for_collisions([self.part1, self.part2])

    def test_collisions_between_two_parts(self):
        raised = self.assertRaises(
            errors.SnapcraftPartConflictError,
            pluginhandler.check_for_collisions,
            [self.part1, self.part2, self.part3],
        )

        self.assertThat(raised.other_part_name, Equals("part2"))
        self.assertThat(raised.part_name, Equals("part3"))
        self.assertThat(raised.file_paths, Equals("    1\n    a/2"))

    def test_collisions_checks_symlinks(self):
        raised = self.assertRaises(
            errors.SnapcraftPartConflictError,
            pluginhandler.check_for_collisions,
            [self.part5, self.part6],
        )

        self.assertThat(
            str(raised),
            Contains(
                "Parts 'part5' and 'part6' have the following files, but with "
                "different contents:\n    a"
            ),
        )

    def test_collisions_not_both_symlinks(self):
        raised = self.assertRaises(
            errors.SnapcraftPartConflictError,
            pluginhandler.check_for_collisions,
            [self.part1, self.part5],
        )

        self.assertThat(
            str(raised),
            Contains(
                "Parts 'part1' and 'part5' have the following files, but with "
                "different contents:\n    a"
            ),
        )

    def test_collisions_between_two_parts_pc_files(self):
        raised = self.assertRaises(
            errors.SnapcraftPartConflictError,
            pluginhandler.check_for_collisions,
            [self.part1, self.part4],
        )

        self.assertThat(raised.other_part_name, Equals("part1"))
        self.assertThat(raised.part_name, Equals("part4"))
        self.assertThat(raised.file_paths, Equals("    file.pc"))

    def test_collision_with_part_not_built(self):
        part_built = self.load_part(
            "part_built", part_properties={"stage": ["collision"]}
        )
        part_built.part_install_dir = "part_built_install"
        # a part built has the stage file in the installdir.
        os.makedirs(part_built.part_install_dir)
        open(os.path.join(part_built.part_install_dir, "collision"), "w").close()
        part_not_built = self.load_part(
            "part_not_built", part_properties={"stage": ["collision"]}
        )
        part_not_built.part_install_dir = "part_not_built_install"
        # a part not built doesn't have the stage file in the installdir.
        pluginhandler.check_for_collisions([part_built, part_not_built])


class StagePackagesTestCase(unit.TestCase):
    def test_missing_stage_package_raises_exception(self):
        fake_repo = Mock()
        fake_repo.fetch_stage_packages.side_effect = repo.errors.PackageNotFoundError(
            "non-existing"
        )
        part = self.load_part(
            "stage-test",
            part_properties={"stage-packages": ["non-existing"]},
            stage_packages_repo=fake_repo,
        )

        raised = self.assertRaises(errors.StagePackageDownloadError, part.prepare_pull)

        self.assertThat(raised.part_name, Equals("stage-test"))
        self.assertThat(
            raised.message, Equals("The package 'non-existing' was not found.")
        )


class FilesetsTestCase(unit.TestCase):
    def test_combine_filesets_explicit_wildcard(self):
        fileset_1 = ["a", "b"]
        fileset_2 = ["*"]

        expected_fileset = ["a", "b"]
        combined_fileset = pluginhandler._combine_filesets(fileset_1, fileset_2)
        self.assertThat(set(combined_fileset), Equals(set(expected_fileset)))

    def test_combine_filesets_implicit_wildcard(self):
        fileset_1 = ["a", "b"]
        fileset_2 = ["-c"]

        expected_fileset = ["a", "-c", "b"]
        combined_fileset = pluginhandler._combine_filesets(fileset_1, fileset_2)
        self.assertThat(set(combined_fileset), Equals(set(expected_fileset)))

    def test_combine_filesets_no_wildcard(self):
        fileset_1 = ["a", "b"]
        fileset_2 = ["a"]

        expected_fileset = ["a"]
        combined_fileset = pluginhandler._combine_filesets(fileset_1, fileset_2)
        self.assertThat(set(combined_fileset), Equals(set(expected_fileset)))

    def test_combine_filesets_with_contradiciton(self):
        fileset_1 = ["-a"]
        fileset_2 = ["a"]

        raised = self.assertRaises(
            errors.PrimeFileConflictError,
            pluginhandler._combine_filesets,
            fileset_1,
            fileset_2,
        )
        self.assertThat(raised.fileset, Equals({"a"}))

    def test_get_includes(self):
        fileset = ["-a", "b"]
        expected_includes = ["b"]

        includes = pluginhandler._get_includes(fileset)
        self.assertThat(set(includes), Equals(set(expected_includes)))

    def test_get_excludes(self):
        fileset = ["-a", "b"]
        expected_excludes = ["a"]

        excludes = pluginhandler._get_excludes(fileset)
        self.assertThat(set(excludes), Equals(set(expected_excludes)))


class SourcesTestCase(unit.TestCase):
    def test_do_not_follow_links(self):
        properties = dict(source=".")
        handler = self.load_part("test-part", part_properties=properties)

        # Create a file and a symlink to it
        open("file", mode="w").close()
        os.symlink("file", "symlinkfile")

        # Create a directory and a symlink to it
        os.mkdir("dir")
        os.symlink("dir", "symlinkdir")

        handler.pull()
        handler.build()

        # Make sure this is still a link
        build_file_path = os.path.join(handler.part_build_dir, "file")
        build_symlinkfile_path = os.path.join(handler.part_build_dir, "symlinkfile")

        self.assertTrue(os.path.isfile(build_file_path))
        self.assertTrue(os.path.islink(build_symlinkfile_path))

        build_dir_path = os.path.join(handler.part_build_dir, "dir")
        build_symlinkdir_path = os.path.join(handler.part_build_dir, "symlinkdir")

        self.assertTrue(os.path.isdir(build_dir_path))
        self.assertTrue(os.path.isdir(build_symlinkdir_path))

    def test_pull_ignores_snapcraft_files_in_source_dir(self):
        properties = dict(source=".")
        handler = self.load_part("test-part", part_properties=properties)

        open("my-snap.snap", "w").close()
        open("my-snap", "w").close()

        handler.pull()

        for file_ in common.SNAPCRAFT_FILES:
            self.assertFalse(
                os.path.exists(os.path.join(handler.part_source_dir, file_))
            )
        self.assertThat(
            os.path.join(handler.part_source_dir, "my-snap.snap"), Not(FileExists())
        )

        # Make sure we don't filter things out incorrectly
        self.assertThat(os.path.join(handler.part_source_dir, "my-snap"), FileExists())

    def test_source_with_unrecognized_source_must_raise_exception(self):
        properties = dict(source="unrecognized://test_source")

        self.assertRaises(
            SnapcraftSourceUnhandledError,
            self.load_part,
            "test-part",
            part_properties=properties,
        )
