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
import stat
import tempfile
from collections import OrderedDict
from textwrap import dedent
from unittest.mock import call, Mock, MagicMock, patch

from testtools.matchers import Contains, Equals, FileExists, Not

import snapcraft
from . import mocks
from snapcraft.internal import (
    common,
    elf,
    errors,
    lifecycle,
    pluginhandler,
    project_loader,
    repo,
    states,
    steps,
)
from snapcraft.internal.sources.errors import SnapcraftSourceUnhandledError
from snapcraft.plugins import nil
from snapcraft.project import Project
from tests import fixture_setup, unit


class PluginTestCase(unit.TestCase):
    def test_build_with_subdir_copies_sourcedir(self):
        handler = self.load_part("test-part", part_properties={"source-subdir": "src"})

        sourcedir = handler.plugin.sourcedir
        source_subdir = handler.plugin.options.source_subdir

        subdir = os.path.join(sourcedir, source_subdir)
        os.makedirs(subdir)
        open(os.path.join(sourcedir, "file1"), "w").close()
        open(os.path.join(subdir, "file2"), "w").close()

        self.assertThat(
            handler.plugin.builddir,
            Equals(os.path.join(handler.plugin.build_basedir, source_subdir)),
        )

        handler.build()

        self.assertTrue(
            os.path.exists(os.path.join(handler.plugin.build_basedir, "file1"))
        )
        self.assertTrue(os.path.exists(os.path.join(handler.plugin.builddir, "file2")))

    def test_build_with_missing_metadata_file(self):
        handler = self.load_part(
            "test-part", part_properties={"parse-info": ["missing-file"]}
        )
        handler.makedirs()

        raised = self.assertRaises(errors.MissingMetadataFileError, handler.build)
        self.assertThat(raised.path, Equals("missing-file"))

    def test_build_without_subdir_copies_sourcedir(self):
        handler = self.load_part("test-part")

        os.makedirs(handler.plugin.sourcedir)
        open(os.path.join(handler.plugin.sourcedir, "file"), "w").close()

        self.assertThat(handler.plugin.builddir, Equals(handler.plugin.build_basedir))

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
            "\-everything",
            r"\\a",
        ]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertThat(
            include, Equals(["opt/something", "usr/bin", "-everything", r"\a"])
        )
        self.assertThat(exclude, Equals(["etc", "usr/lib/*.a"]))

    @patch.object(snapcraft.plugins.nil.NilPlugin, "snap_fileset")
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

    @patch("os.chown")
    def test_migrate_files_preserves_ownership(self, chown_mock):
        os.makedirs("install")
        os.makedirs("stage")

        foo = os.path.join("install", "foo")

        with open(foo, "w") as f:
            f.write("installed")

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        self.assertTrue(chown_mock.called)

    @patch("os.chown")
    def test_migrate_files_chown_permissions(self, chown_mock):
        os.makedirs("install")
        os.makedirs("stage")

        chown_mock.side_effect = PermissionError("No no no")

        foo = os.path.join("install", "foo")

        with open(foo, "w") as f:
            f.write("installed")

        files, dirs = pluginhandler._migratable_filesets(["*"], "install")
        pluginhandler._migrate_files(
            files, dirs, "install", "stage", follow_symlinks=True
        )

        self.assertTrue(chown_mock.called)

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

    @patch("os.chown")
    def test_migrate_files_preserves_file_mode_chown_permissions(self, chown_mock):
        chown_mock.side_effect = PermissionError("No no no")
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

        self.assertTrue(chown_mock.called)

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


class MigratePluginTestCase(unit.TestCase):

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

    def test_migrate_snap_files(self):
        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        srcdir = tmpdir + "/install"
        os.makedirs(tmpdir + "/install/1/1a/1b")
        os.makedirs(tmpdir + "/install/2/2a")
        os.makedirs(tmpdir + "/install/3")
        open(tmpdir + "/install/a", mode="w").close()
        open(tmpdir + "/install/b", mode="w").close()
        open(tmpdir + "/install/1/a", mode="w").close()
        open(tmpdir + "/install/3/a", mode="w").close()

        dstdir = tmpdir + "/stage"
        os.makedirs(dstdir)

        files, dirs = pluginhandler._migratable_filesets(self.fileset, srcdir)
        pluginhandler._migrate_files(files, dirs, srcdir, dstdir)

        expected = []
        for item in self.result:
            expected.append(os.path.join(tmpdir, item))
        expected.sort()

        result = []
        for root, subdirs, files in os.walk(dstdir):
            for item in files:
                result.append(os.path.join(root, item))
            for item in subdirs:
                result.append(os.path.join(root, item))
        result.sort()

        self.assertThat(result, Equals(expected))


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


class OrganizeTestCase(unit.TestCase):

    scenarios = [
        (
            "simple_file",
            dict(
                setup_dirs=[],
                setup_files=["foo"],
                organize_set={"foo": "bar"},
                expected=[(["bar"], "")],
            ),
        ),
        (
            "simple_dir_with_file",
            dict(
                setup_dirs=["foodir"],
                setup_files=[os.path.join("foodir", "foo")],
                organize_set={"foodir": "bardir"},
                expected=[(["bardir"], ""), (["foo"], "bardir")],
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
            ),
        ),
        (
            "leading_slash_in_value",
            dict(
                setup_dirs=[],
                setup_files=["foo"],
                organize_set={"foo": "/bar"},
                expected=[(["bar"], "")],
            ),
        ),
        (
            "overwrite_existing_file",
            dict(
                setup_dirs=[],
                setup_files=["foo", "bar"],
                organize_set={"foo": "bar"},
                expected=errors.SnapcraftEnvironmentError,
            ),
        ),
        (
            "*_for_files",
            dict(
                setup_dirs=[],
                setup_files=["foo.conf", "bar.conf"],
                organize_set={"*.conf": "dir/"},
                expected=[(["dir"], ""), (["bar.conf", "foo.conf"], "dir")],
            ),
        ),
        (
            "*_for_files_with_non_dir_dst",
            dict(
                setup_dirs=[],
                setup_files=["foo.conf", "bar.conf"],
                organize_set={"*.conf": "dir"},
                expected=errors.SnapcraftEnvironmentError,
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
            ),
        ),
    ]

    def test_organize_file(self):
        base_dir = "install"
        os.makedirs(base_dir)

        for directory in self.setup_dirs:
            os.makedirs(os.path.join(base_dir, directory))

        for file_entry in self.setup_files:
            with open(os.path.join(base_dir, file_entry), "w") as f:
                f.write(file_entry)

        if isinstance(self.expected, type) and issubclass(self.expected, Exception):
            self.assertRaises(
                self.expected,
                pluginhandler._organize_filesets,
                self.organize_set,
                base_dir,
            )
        else:
            pluginhandler._organize_filesets(self.organize_set, base_dir)
            for expect in self.expected:
                dir_path = os.path.join(base_dir, expect[1])
                dir_contents = os.listdir(dir_path)
                dir_contents.sort()
                self.assertThat(dir_contents, Equals(expect[0]))


class RealStageTestCase(unit.TestCase):
    def make_snapcraft_project(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: pc-file-test
            version: 1.0
            summary: test pkg-config .pc
            description: when the .pc files reach stage the should be reprefixed
            confinement: strict
            grade: stable

            parts:
                stage-pc:
                    plugin: nil
        """
            )
        )  # noqa: E501
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


class PluginMakedirsTestCase(unit.TestCase):

    scenarios = [
        ("existing_dirs", {"make_dirs": True}),
        ("unexisting_dirs", {"make_dirs": False}),
    ]

    def get_plugin_dirs(self, part_name):
        return [
            os.path.join(self.parts_dir, part_name, "src"),
            os.path.join(self.parts_dir, part_name, "build"),
            os.path.join(self.parts_dir, part_name, "install"),
            os.path.join(self.stage_dir),
            os.path.join(self.prime_dir),
        ]

    def test_makedirs_with_existing_dirs(self):
        part_name = "test_part"
        dirs = self.get_plugin_dirs(part_name)
        if self.make_dirs:
            os.makedirs(os.path.join(self.parts_dir, part_name))
            for d in dirs:
                os.mkdir(d)

        p = self.load_part(part_name)
        p.makedirs()
        for d in dirs:
            self.assertTrue(os.path.exists(d), "{} does not exist".format(d))


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


class StateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = patch("snapcraft._baseplugin.BasePlugin.get_pull_properties")
        self.get_pull_properties_mock = patcher.start()
        self.get_pull_properties_mock.return_value = []
        self.addCleanup(patcher.stop)

        patcher = patch("snapcraft._baseplugin.BasePlugin.get_build_properties")
        self.get_build_properties_mock = patcher.start()
        self.get_build_properties_mock.return_value = []
        self.addCleanup(patcher.stop)

        self.handler = self.load_part("test_part")

        self.handler.makedirs()

        patcher = patch("snapcraft.internal.elf.get_elf_files")
        self.get_elf_files_mock = patcher.start()
        self.get_elf_files_mock.return_value = frozenset()
        self.addCleanup(patcher.stop)


class StateTestCase(StateBaseTestCase):
    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state(self, repo_mock):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertTrue(expected in state.properties)
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state_with_extracted_metadata(self, repo_mock):
        self.handler = self.load_part(
            "test_part", part_properties={"parse-info": ["metadata-file"]}
        )

        # Create metadata file
        open("metadata-file", "w").close()

        def _fake_extractor(file_path):
            return snapcraft.extractors.ExtractedMetadata(
                common_id="test_common_id",
                summary="test summary",
                description="test description",
                icon="/test/path",
                desktop_file_paths=[
                    "usr/share/applications/com.example.test/app.desktop"
                ],
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(state.project_options, Contains("deb_arch"))

        self.assertTrue(type(state.extracted_metadata) is OrderedDict)
        for expected in ("metadata", "files"):
            self.assertThat(state.extracted_metadata, Contains(expected))
        metadata = state.extracted_metadata["metadata"]
        self.assertThat(metadata.get_common_id(), Equals("test_common_id"))
        self.assertThat(metadata.get_summary(), Equals("test summary"))
        self.assertThat(metadata.get_description(), Equals("test description"))
        self.assertThat(metadata.get_icon(), Equals("/test/path"))
        self.assertThat(
            metadata.get_desktop_file_paths(),
            Equals(["usr/share/applications/com.example.test/app.desktop"]),
        )
        files = state.extracted_metadata["files"]
        self.assertThat(files, Equals(["metadata-file"]))

    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state_with_scriptlet_metadata(self, repo_mock):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-pull": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-pull"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_pull_state_with_properties(self):
        self.get_pull_properties_mock.return_value = ["foo"]
        self.handler.plugin.options.foo = "bar"
        self.handler._part_properties = {"foo": "bar"}

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertTrue("foo" in state.properties)
        self.assertThat(state.properties["foo"], Equals("bar"))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch.object(nil.NilPlugin, "clean_pull")
    def test_clean_pull_state(self, mock_clean_pull):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()

        self.handler.clean_pull()

        # Verify that the plugin had clean_pull() called
        mock_clean_pull.assert_called_once_with()

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

    def test_build_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(9))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "prepare",
            "build",
            "install",
            "override-build",
        ]:
            self.assertTrue(expected in state.properties)
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    def test_build_state_with_extracted_metadata(self):
        self.handler = self.load_part(
            "test_part", part_properties={"parse-info": ["metadata-file"]}
        )

        # Create metadata file
        open(os.path.join(self.handler.plugin.sourcedir, "metadata-file"), "w").close()

        def _fake_extractor(file_path):
            return snapcraft.extractors.ExtractedMetadata(
                common_id="test_common_id",
                summary="test summary",
                description="test description",
                icon="/test/path",
                desktop_file_paths=[
                    "usr/share/applications/com.example.test/app.desktop"
                ],
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(9))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "prepare",
            "build",
            "install",
            "override-build",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(state.project_options, Contains("deb_arch"))

        self.assertTrue(type(state.extracted_metadata) is OrderedDict)
        for expected in ("metadata", "files"):
            self.assertThat(state.extracted_metadata, Contains(expected))
        metadata = state.extracted_metadata["metadata"]
        self.assertThat(metadata.get_common_id(), Equals("test_common_id"))
        self.assertThat(metadata.get_summary(), Equals("test summary"))
        self.assertThat(metadata.get_description(), Equals("test description"))
        self.assertThat(metadata.get_icon(), Equals("/test/path"))
        self.assertThat(
            metadata.get_desktop_file_paths(),
            Equals(["usr/share/applications/com.example.test/app.desktop"]),
        )
        files = state.extracted_metadata["files"]
        self.assertThat(files, Equals(["metadata-file"]))

    def test_build_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-build": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(9))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "prepare",
            "build",
            "install",
            "override-build",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-build"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_build_state_with_properties(self):
        self.get_build_properties_mock.return_value = ["foo"]
        self.handler.plugin.options.foo = "bar"
        self.handler._part_properties = {"foo": "bar"}

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertTrue("foo" in state.properties)
        self.assertThat(state.properties["foo"], Equals("bar"))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch.object(nil.NilPlugin, "clean_build")
    def test_clean_build_state(self, mock_clean_build):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)

        self.handler.mark_done(steps.PULL)
        self.handler.build()

        self.handler.clean_build()

        # Verify that the plugin had clean_build() called
        mock_clean_build.assert_called_once_with()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))

    def test_stage_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertTrue("stage" in state.properties)
        self.assertThat(state.properties["stage"], Equals(["*"]))
        self.assertTrue("filesets" in state.properties)
        self.assertThat(state.properties["filesets"], Equals({}))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_stage_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-stage": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(3))
        for expected in ["stage", "filesets", "override-stage"]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-stage"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_stage_state_with_stage_keyword(self):
        self.handler.plugin.options.stage = ["bin/1"]
        self.handler._part_properties = {"stage": ["bin/1"]}

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertTrue("stage" in state.properties)
        self.assertThat(state.properties["stage"], Equals(["bin/1"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))

    def test_clean_stage_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(bindir))

    def test_clean_stage_state_multiple_parts(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()
        open(os.path.join(bindir, "3"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertFalse(os.path.exists(os.path.join(bindir, "2")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "3")),
            "Expected 'bin/3' to remain as it wasn't staged by this part",
        )

    def test_clean_stage_state_common_files(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({"other_part": states.StageState({"bin/2"}, {"bin"})})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "2")),
            "Expected 'bin/2' to remain as it's required by other parts",
        )

    def test_clean_stage_old_state(self):
        self.handler.mark_done(steps.STAGE, None)
        raised = self.assertRaises(
            errors.MissingStateCleanError, self.handler.clean_stage, {}
        )

        self.assertThat(raised.step, Equals(steps.STAGE))

    @patch("shutil.copy")
    def test_prime_state(self, mock_copy):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/1", "bin/2"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_prime_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-prime": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        state = self.handler.get_prime_state()

        self.assertTrue(state, "Expected prime to save state YAML")
        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(2))
        for expected in ["prime", "override-prime"]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-prime"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    @patch("shutil.copy")
    def test_prime_state_with_stuff_already_primed(self, mock_copy):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        bindir = os.path.join(self.handler.primedir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        # bin/2 shouldn't be in this list as it was already primed by another
        # part.
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/1"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    @patch(
        "snapcraft.internal.elf.ElfFile._extract",
        return_value=(("", "", ""), "EXEC", "", dict(), False),
    )
    @patch("snapcraft.internal.elf.ElfFile.load_dependencies")
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_with_dependencies(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        mock_load_dependencies.return_value = {
            "/foo/bar/baz",
            "{}/lib1/installed".format(self.handler.plugin.installdir),
            "{}/lib2/staged".format(self.handler.stagedir),
        }
        self.get_elf_files_mock.return_value = frozenset(
            [
                elf.ElfFile(path=os.path.join(self.handler.primedir, "bin", "1")),
                elf.ElfFile(path=os.path.join(self.handler.primedir, "bin", "2")),
            ]
        )
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/1", "bin/2"}
        )
        mock_migrate_files.assert_has_calls(
            [
                call(
                    {"bin/1", "bin/2"},
                    {"bin"},
                    self.handler.stagedir,
                    self.handler.primedir,
                ),
                call(
                    {"foo/bar/baz"},
                    {"foo/bar"},
                    "/",
                    self.handler.primedir,
                    follow_symlinks=True,
                ),
            ]
        )

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(3))
        self.assertTrue("foo/bar" in state.dependency_paths)
        self.assertTrue("lib1" in state.dependency_paths)
        self.assertTrue("lib2" in state.dependency_paths)
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    @patch(
        "snapcraft.internal.elf.ElfFile._extract",
        return_value=(("", "", ""), "EXEC", "", dict(), False),
    )
    @patch("snapcraft.internal.elf.ElfFile.load_dependencies")
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_disable_ldd_crawl(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        # Disable system library migration (i.e. ldd crawling).
        self.handler = self.load_part(
            "test_part", part_properties={"build-attributes": ["no-system-libraries"]}
        )

        self.get_elf_files_mock.return_value = frozenset(
            [elf.ElfFile(path=os.path.join(self.handler.primedir, "bin", "file"))]
        )
        # Pretend we found a system dependency, as well as a part and stage
        # dependency.
        mock_load_dependencies.return_value = set(
            [
                "/foo/bar/baz",
                "{}/lib1/installed".format(self.handler.plugin.installdir),
                "{}/lib2/staged".format(self.handler.stagedir),
            ]
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "file"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        mock_migrate_files.reset_mock()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/file"}
        )
        # Verify that only the part's files were migrated-- not the system
        # dependency.
        mock_migrate_files.assert_called_once_with(
            {"bin/file"}, {"bin"}, self.handler.stagedir, self.handler.primedir
        )

        state = self.handler.get_prime_state()

        # Verify that only the part and staged libraries were saved into the
        # dependency paths, not the system dependency.
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertThat(len(state.dependency_paths), Equals(2))
        self.assertTrue("lib1" in state.dependency_paths)
        self.assertTrue("lib2" in state.dependency_paths)

    @patch(
        "snapcraft.internal.elf.ElfFile._extract",
        return_value=(("", "", ""), "EXEC", "", dict(), False),
    )
    @patch(
        "snapcraft.internal.elf.ElfFile.load_dependencies",
        return_value=set(["/foo/bar/baz"]),
    )
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_with_shadowed_dependencies(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        self.get_elf_files_mock.return_value = frozenset([elf.ElfFile(path="bin/1")])
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        foobardir = os.path.join(self.handler.plugin.installdir, "foo", "bar")
        os.makedirs(bindir)
        os.makedirs(foobardir)

        # Make a "binary" as well as a "library" at the same path as the one on
        # the system
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(foobardir, "baz"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        mock_migrate_files.reset_mock()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/1", "foo/bar/baz"}
        )
        mock_migrate_files.assert_called_once_with(
            {"bin/1", "foo/bar/baz"},
            {"bin", "foo", "foo/bar"},
            self.handler.stagedir,
            self.handler.primedir,
        )

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertThat(len(state.dependency_paths), Equals(1))
        self.assertTrue("foo/bar" in state.dependency_paths)

    @patch("shutil.copy")
    def test_prime_state_with_prime_keyword(self, mock_copy):
        self.handler = self.load_part("test_part", part_properties={"prime": ["bin/1"]})

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler.primedir, {"bin/1"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["bin/1"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_clean_prime_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(bindir))

    def test_clean_prime_state_multiple_parts(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()
        open(os.path.join(bindir, "3"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertFalse(os.path.exists(os.path.join(bindir, "2")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "3")),
            "Expected 'bin/3' to remain as it wasn't primed by this part",
        )

    def test_clean_prime_state_common_files(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({"other_part": states.PrimeState({"bin/2"}, {"bin"})})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "2")),
            "Expected 'bin/2' to remain as it's required by other parts",
        )

    def test_clean_prime_old_state(self):
        self.handler.mark_done(steps.PRIME, None)
        raised = self.assertRaises(
            errors.MissingStateCleanError, self.handler.clean_prime, {}
        )

        self.assertThat(raised.step, Equals(steps.PRIME))


class StateFileMigrationTestCase(StateBaseTestCase):

    scenarios = [(step.name, dict(step=step)) for step in steps.STEPS]

    def test_state_file_migration(self):
        part_name = "foo"
        shutil.rmtree(self.parts_dir)
        part_dir = os.path.join(self.parts_dir, part_name)
        os.makedirs(part_dir)
        with open(os.path.join(part_dir, "state"), "w") as f:
            f.write(self.step.name)

        handler = self.load_part(part_name)
        self.assertThat(handler.latest_step(), Equals(self.step))


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
        project_options = snapcraft.ProjectOptions(target_deb_arch="amd64")
        self.handler = self.load_part("test-part", project_options=project_options)
        self.handler.mark_build_done()
        self.assertFalse(
            self.handler.is_clean(steps.BUILD), "Build step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.BUILD), "Build step was unexpectedly dirty"
        )

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project_options = snapcraft.ProjectOptions(target_deb_arch="armhf")
        self.handler = self.load_part("test-part", project_options=project_options)
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
        project_options = snapcraft.ProjectOptions(target_deb_arch="amd64")
        self.handler = self.load_part("test-part", project_options=project_options)
        self.handler.mark_pull_done()
        self.assertFalse(
            self.handler.is_clean(steps.PULL), "Pull step was unexpectedly clean"
        )
        self.assertFalse(
            self.handler.is_dirty(steps.PULL), "Pull step was unexpectedly dirty"
        )

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project_options = snapcraft.ProjectOptions(target_deb_arch="armhf")
        self.handler = self.load_part("test-part", project_options=project_options)
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

        self.handler = self.load_part("test-part")
        self.handler.makedirs()

    def set_modified_time_later(self, target, reference):
        access_time = os.stat(reference).st_atime
        modified_time = os.stat(reference).st_atime
        os.utime(target, (access_time, modified_time + 1))

    def test_prime_is_outdated(self):
        self.handler.mark_prime_done(set(), set(), set())
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


class CleanBaseTestCase(unit.TestCase):
    def clear_common_directories(self):
        if os.path.exists(self.parts_dir):
            shutil.rmtree(self.parts_dir)

        if os.path.exists(self.stage_dir):
            shutil.rmtree(self.stage_dir)

        if os.path.exists(self.prime_dir):
            shutil.rmtree(self.prime_dir)


class CleanTestCase(CleanBaseTestCase):
    @patch.object(pluginhandler.PluginHandler, "is_clean")
    @patch("os.rmdir")
    @patch("os.listdir")
    def test_clean_part_that_exists(self, mock_listdir, mock_rmdir, mock_is_clean):
        mock_listdir.return_value = False
        mock_is_clean.return_value = True

        part_name = "test_part"
        partdir = os.path.join(self.parts_dir, part_name)
        os.makedirs(partdir)

        p = self.load_part(part_name)
        p.clean()

        mock_listdir.assert_called_once_with(partdir)
        mock_rmdir.assert_called_once_with(partdir)

    @patch("os.rmdir")
    @patch("os.listdir")
    @patch("os.path.exists")
    def test_clean_part_already_clean(self, mock_exists, mock_listdir, mock_rmdir):
        mock_exists.return_value = False

        part_name = "test_part"
        p = self.load_part(part_name)
        p.clean()

        partdir = os.path.join(self.parts_dir, part_name)
        mock_exists.assert_has_calls([call(partdir)])
        self.assertFalse(mock_listdir.called)
        self.assertFalse(mock_rmdir.called)

    @patch.object(pluginhandler.PluginHandler, "is_clean")
    @patch("os.rmdir")
    @patch("os.listdir")
    def test_clean_part_remaining_parts(self, mock_listdir, mock_rmdir, mock_is_clean):
        mock_listdir.return_value = True
        mock_is_clean.return_value = True

        part_name = "test_part"
        partdir = os.path.join(self.parts_dir, part_name)
        os.makedirs(partdir)

        p = self.load_part(part_name)
        p.clean()

        mock_listdir.assert_called_once_with(partdir)
        self.assertFalse(mock_rmdir.called)

    def test_clean_prime_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = self.load_part("part1")
        handler1.makedirs()

        bindir = os.path.join(handler1.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()

        handler1.mark_done(steps.BUILD)

        # Now create part2 and get it through the "build" step.
        handler2 = self.load_part("part2")
        handler2.makedirs()

        bindir = os.path.join(handler2.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        handler2.mark_done(steps.BUILD)

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # And prime both parts
        handler1.prime()
        handler2.prime()

        # Verify that part1's file has been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "1")))

        # Verify that part2's file has been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "2")))

        # Now clean the prime step for part1
        handler1.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "1")),
            "Expected part1's primeped files to be cleaned",
        )

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.prime_dir, "bin", "2")),
            "Expected part2's primeped files to be untouched",
        )

    def test_clean_prime_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = self.load_part("part1")
        handler.makedirs()

        bindir = os.path.join(handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        handler.mark_done(steps.BUILD)
        handler.stage()
        handler.prime()

        # Verify that both files have been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "1")))
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "2")))

        # Now update the `snap` fileset to only snap one of these files
        handler.plugin.options.snap = ["bin/1"]

        # Now clean the prime step for part1
        handler.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "1")),
            "Expected bin/1 to be cleaned",
        )
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "2")),
            "Expected bin/2 to be cleaned as well, even though the filesets "
            "changed since it was primeped.",
        )

    def test_clean_old_prime_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        open(os.path.join(self.prime_dir, "1"), "w").close()

        handler.mark_done(steps.PRIME, None)

        self.assertTrue(os.path.exists(handler.plugin.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.plugin.partdir))

    def test_clean_prime_old_prime_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        primed_file = os.path.join(self.prime_dir, "1")
        open(primed_file, "w").close()

        handler.mark_done(steps.PRIME, None)

        raised = self.assertRaises(
            errors.MissingStateCleanError, handler.clean, step=steps.PRIME
        )

        self.assertThat(raised.step, Equals(steps.PRIME))
        self.assertTrue(os.path.isfile(primed_file))

    def test_clean_stage_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = self.load_part("part1")
        handler1.makedirs()

        bindir = os.path.join(handler1.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()

        handler1.mark_done(steps.BUILD)

        # Now create part2 and get it through the "build" step.
        handler2 = self.load_part("part2")
        handler2.makedirs()

        bindir = os.path.join(handler2.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        handler2.mark_done(steps.BUILD)

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # Verify that part1's file has been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "1")))

        # Verify that part2's file has been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "2")))

        # Now clean the stage step for part1
        handler1.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "1")),
            "Expected part1's staged files to be cleaned",
        )

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, "bin", "2")),
            "Expected part2's staged files to be untouched",
        )

    def test_clean_stage_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = self.load_part("part1")
        handler.makedirs()

        bindir = os.path.join(handler.plugin.installdir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        handler.mark_done(steps.BUILD)
        handler.stage()

        # Verify that both files have been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "1")))
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "2")))

        # Now update the `stage` fileset to only snap one of these files
        handler.plugin.options.stage = ["bin/1"]

        # Now clean the prime step for part1
        handler.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "1")),
            "Expected bin/1 to be cleaned",
        )
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "2")),
            "Expected bin/2 to be cleaned as well, even though the filesets "
            "changed since it was staged.",
        )

    def test_clean_old_stage_state(self):
        handler = self.load_part("part1")
        handler.makedirs()

        open(os.path.join(self.stage_dir, "1"), "w").close()

        handler.mark_done(steps.STAGE, None)

        self.assertTrue(os.path.exists(handler.plugin.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.plugin.partdir))

    def test_clean_stage_old_stage_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        staged_file = os.path.join(self.stage_dir, "1")
        open(staged_file, "w").close()

        handler.mark_done(steps.STAGE, None)

        raised = self.assertRaises(
            errors.MissingStateCleanError, handler.clean, step=steps.STAGE
        )

        self.assertThat(raised.step, Equals(steps.STAGE))
        self.assertTrue(os.path.isfile(staged_file))


class CleanPrimeTestCase(CleanBaseTestCase):

    scenarios = [
        ("all", {"fileset": ["*"]}),
        ("no1", {"fileset": ["-1"]}),
        ("onlya", {"fileset": ["a"]}),
        ("onlybase", {"fileset": ["*", "-*/*"]}),
        ("only1a", {"fileset": ["1/a"]}),
        ("nostara", {"fileset": ["-*/a"]}),
    ]

    def test_clean_prime(self):
        self.clear_common_directories()

        handler = self.load_part("test_part", part_properties={"prime": self.fileset})
        handler.makedirs()

        installdir = handler.plugin.installdir
        os.makedirs(installdir + "/1/1a/1b")
        os.makedirs(installdir + "/2/2a")
        os.makedirs(installdir + "/3")
        open(installdir + "/a", mode="w").close()
        open(installdir + "/b", mode="w").close()
        open(installdir + "/1/a", mode="w").close()
        open(installdir + "/3/a", mode="w").close()

        handler.mark_done(steps.BUILD)

        # Stage the installed files
        handler.stage()

        # Now prime them
        handler.prime()

        self.assertTrue(os.listdir(self.prime_dir))

        handler.clean_prime({})

        self.assertFalse(
            os.listdir(self.prime_dir), "Expected prime dir to be completely cleaned"
        )


class CleanStageTestCase(CleanBaseTestCase):

    scenarios = [
        ("all", {"fileset": ["*"]}),
        ("no1", {"fileset": ["-1"]}),
        ("onlya", {"fileset": ["a"]}),
        ("onlybase", {"fileset": ["*", "-*/*"]}),
        ("only1a", {"fileset": ["1/a"]}),
        ("nostara", {"fileset": ["-*/a"]}),
    ]

    def test_clean_stage(self):
        self.clear_common_directories()

        handler = self.load_part("test_part", part_properties={"stage": self.fileset})
        handler.makedirs()

        installdir = handler.plugin.installdir
        os.makedirs(installdir + "/1/1a/1b")
        os.makedirs(installdir + "/2/2a")
        os.makedirs(installdir + "/3")
        open(installdir + "/a", mode="w").close()
        open(installdir + "/b", mode="w").close()
        open(installdir + "/1/a", mode="w").close()
        open(installdir + "/3/a", mode="w").close()

        handler.mark_done(steps.BUILD)

        # Stage the installed files
        handler.stage()

        self.assertTrue(os.listdir(self.stage_dir))

        handler.clean_stage({})

        self.assertFalse(
            os.listdir(self.stage_dir), "Expected stage dir to be completely cleaned"
        )


class PerStepCleanTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.manager_mock = MagicMock()

        patcher = patch.object(pluginhandler.PluginHandler, "clean_pull")
        self.manager_mock.attach_mock(patcher.start(), "clean_pull")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_build")
        self.manager_mock.attach_mock(patcher.start(), "clean_build")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_stage")
        self.manager_mock.attach_mock(patcher.start(), "clean_stage")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_prime")
        self.manager_mock.attach_mock(patcher.start(), "clean_prime")
        self.addCleanup(patcher.stop)

        self.handler = self.load_part("test_part")

    def test_clean_pull_order(self):
        self.handler.clean(step=steps.PULL)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(4))
        self.manager_mock.assert_has_calls(
            [
                call.clean_prime({}),
                call.clean_stage({}),
                call.clean_build(),
                call.clean_pull(),
            ]
        )

    def test_clean_build_order(self):
        self.handler.clean(step=steps.BUILD)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(3))
        self.manager_mock.assert_has_calls(
            [call.clean_prime({}), call.clean_stage({}), call.clean_build()]
        )

    def test_clean_stage_order(self):
        self.handler.clean(step=steps.STAGE)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(2))
        self.manager_mock.assert_has_calls([call.clean_prime({}), call.clean_stage({})])

    def test_clean_prime_order(self):
        self.handler.clean(step=steps.PRIME)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(1))
        self.manager_mock.assert_has_calls([call.clean_prime({})])


class CollisionTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = self.load_part("part1")
        part1.plugin.installdir = tmpdir + "/install1"
        os.makedirs(part1.plugin.installdir + "/a")
        open(part1.plugin.installdir + "/a/1", mode="w").close()
        with open(part1.plugin.installdir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part1.plugin.installdir))
            f.write("Name: File\n")

        part2 = self.load_part("part2")
        part2.plugin.installdir = tmpdir + "/install2"
        os.makedirs(part2.plugin.installdir + "/a")
        with open(part2.plugin.installdir + "/1", mode="w") as f:
            f.write("1")
        open(part2.plugin.installdir + "/2", mode="w").close()
        with open(part2.plugin.installdir + "/a/2", mode="w") as f:
            f.write("a/2")
        with open(part2.plugin.installdir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part2.plugin.installdir))
            f.write("Name: File\n")

        part3 = self.load_part("part3")
        part3.plugin.installdir = tmpdir + "/install3"
        os.makedirs(part3.plugin.installdir + "/a")
        os.makedirs(part3.plugin.installdir + "/b")
        with open(part3.plugin.installdir + "/1", mode="w") as f:
            f.write("2")
        with open(part2.plugin.installdir + "/2", mode="w") as f:
            f.write("1")
        open(part3.plugin.installdir + "/a/2", mode="w").close()

        part4 = self.load_part("part4")
        part4.plugin.installdir = tmpdir + "/install4"
        os.makedirs(part4.plugin.installdir)
        with open(part4.plugin.installdir + "/file.pc", mode="w") as f:
            f.write("prefix={}\n".format(part4.plugin.installdir))
            f.write("Name: ConflictFile\n")

        # Create a new part with a symlink that collides with part1's
        # non-symlink.
        part5 = self.load_part("part5")
        part5.plugin.installdir = os.path.join(tmpdir, "install5")
        os.makedirs(part5.plugin.installdir)
        os.symlink("foo", os.path.join(part5.plugin.installdir, "a"))

        # Create a new part with a symlink that points to a different place
        # than part5's symlink.
        part6 = self.load_part("part6")
        part6.plugin.installdir = os.path.join(tmpdir, "install6")
        os.makedirs(part6.plugin.installdir)
        os.symlink("bar", os.path.join(part6.plugin.installdir, "a"))

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
        part_built.plugin.installdir = "part_built_install"
        # a part built has the stage file in the installdir.
        os.makedirs(part_built.plugin.installdir)
        open(os.path.join(part_built.plugin.installdir, "collision"), "w").close()
        part_not_built = self.load_part(
            "part_not_built", part_properties={"stage": ["collision"]}
        )
        part_not_built.plugin.installdir = "part_not_built_install"
        # a part not built doesn't have the stage file in the installdir.
        pluginhandler.check_for_collisions([part_built, part_not_built])


class StagePackagesTestCase(unit.TestCase):
    def test_missing_stage_package_raises_exception(self):
        fake_repo = Mock()
        fake_repo.get.side_effect = repo.errors.PackageNotFoundError("non-existing")
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
        build_file_path = os.path.join(handler.plugin.builddir, "file")
        build_symlinkfile_path = os.path.join(handler.plugin.builddir, "symlinkfile")

        self.assertTrue(os.path.isfile(build_file_path))
        self.assertTrue(os.path.islink(build_symlinkfile_path))

        build_dir_path = os.path.join(handler.plugin.builddir, "dir")
        build_symlinkdir_path = os.path.join(handler.plugin.builddir, "symlinkdir")

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
                os.path.exists(os.path.join(handler.plugin.sourcedir, file_))
            )
        self.assertThat(
            os.path.join(handler.plugin.sourcedir, "my-snap.snap"), Not(FileExists())
        )

        # Make sure we don't filter things out incorrectly
        self.assertThat(os.path.join(handler.plugin.sourcedir, "my-snap"), FileExists())

    def test_source_with_unrecognized_source_must_raise_exception(self):
        properties = dict(source="unrecognized://test_source")

        self.assertRaises(
            SnapcraftSourceUnhandledError,
            self.load_part,
            "test-part",
            part_properties=properties,
        )


class CleanPullTestCase(unit.TestCase):
    def test_clean_pull_directory(self):
        handler = self.load_part("test-part")

        handler.pull()
        source_file = os.path.join(handler.plugin.sourcedir, "source")
        open(source_file, "w").close()

        handler.clean_pull()

        # The source directory should now be gone
        self.assertFalse(os.path.exists(handler.plugin.sourcedir))

    def test_clean_pull_symlink(self):
        real_source_directory = os.path.join(os.getcwd(), "src")
        os.mkdir(real_source_directory)

        handler = self.load_part("test-part", part_properties={"source": "src"})

        handler.pull()
        os.rmdir(handler.plugin.sourcedir)
        os.symlink(real_source_directory, handler.plugin.sourcedir)

        handler.clean_pull()

        # The source symlink should now be gone, but the real source should
        # still be there.
        self.assertFalse(os.path.exists(handler.plugin.sourcedir))
        self.assertTrue(os.path.isdir(real_source_directory))


class CleanBuildTestCase(unit.TestCase):
    def test_clean_build(self):
        handler = self.load_part("test-part")

        handler.build()

        source_file = os.path.join(handler.plugin.sourcedir, "source")
        open(source_file, "w").close()
        open(os.path.join(handler.plugin.build_basedir, "built"), "w").close()
        open(os.path.join(handler.plugin.installdir, "installed"), "w").close()

        handler.clean_build()

        # Make sure the source file hasn't been touched
        self.assertTrue(os.path.isfile(source_file))

        # Make sure the build directory is gone
        self.assertFalse(os.path.exists(handler.plugin.build_basedir))

        # Make sure the install directory is gone
        self.assertFalse(os.path.exists(handler.plugin.installdir))
