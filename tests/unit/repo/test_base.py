# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import stat
from textwrap import dedent

from testtools.matchers import Equals, FileContains, FileExists, Not

from snapcraft.internal.repo._base import BaseRepo, get_pkg_name_parts
from tests import unit

from . import RepoBaseTestCase


class FixXmlToolsTestCase(RepoBaseTestCase):
    def assert_fix(self, files):
        for test_file in files:
            path = test_file["path"]
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                f.write(test_file["content"])

        BaseRepo.normalize("root")

        for test_file in files:
            self.assertThat(test_file["path"], FileContains(test_file["expected"]))

    def test_fix_xml2_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xml2-config"),
                    "content": "prefix=/usr/foo",
                    "expected": "prefix=root/usr/foo",
                }
            ]
        )

    def test_no_fix_xml2_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xml2-config"),
                    "content": "prefix=/foo",
                    "expected": "prefix=/foo",
                }
            ]
        )

    def test_fix_xslt_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xslt-config"),
                    "content": "prefix=/usr/foo",
                    "expected": "prefix=root/usr/foo",
                }
            ]
        )

    def test_no_fix_xslt_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xslt-config"),
                    "content": "prefix=/foo",
                    "expected": "prefix=/foo",
                }
            ]
        )

    def test_fix_xml2_xslt_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xml2-config"),
                    "content": "prefix=/usr/foo",
                    "expected": "prefix=root/usr/foo",
                },
                {
                    "path": os.path.join("root", "usr", "bin", "xslt-config"),
                    "content": "prefix=/usr/foo",
                    "expected": "prefix=root/usr/foo",
                },
            ]
        )

    def test_no_fix_xml2_xslt_config(self):
        self.assert_fix(
            [
                {
                    "path": os.path.join("root", "usr", "bin", "xml2-config"),
                    "content": "prefix=/foo",
                    "expected": "prefix=/foo",
                },
                {
                    "path": os.path.join("root", "usr", "bin", "xslt-config"),
                    "content": "prefix=/foo",
                    "expected": "prefix=/foo",
                },
            ]
        )


class FixShebangTestCase(RepoBaseTestCase):

    scenarios = [
        (
            "python bin dir",
            {
                "file_path": os.path.join("root", "bin", "a"),
                "content": "#!/usr/bin/python\nimport this",
                "expected": "#!/usr/bin/env python\nimport this",
            },
        ),
        (
            "python3 bin dir",
            {
                "file_path": os.path.join("root", "bin", "d"),
                "content": "#!/usr/bin/python3\nimport this",
                "expected": "#!/usr/bin/env python3\nimport this",
            },
        ),
        (
            "sbin dir",
            {
                "file_path": os.path.join("root", "sbin", "b"),
                "content": "#!/usr/bin/python\nimport this",
                "expected": "#!/usr/bin/env python\nimport this",
            },
        ),
        (
            "usr/bin dir",
            {
                "file_path": os.path.join("root", "usr", "bin", "c"),
                "content": "#!/usr/bin/python\nimport this",
                "expected": "#!/usr/bin/env python\nimport this",
            },
        ),
        (
            "usr/sbin dir",
            {
                "file_path": os.path.join("root", "usr", "sbin", "d"),
                "content": "#!/usr/bin/python\nimport this",
                "expected": "#!/usr/bin/env python\nimport this",
            },
        ),
        (
            "opt/bin dir",
            {
                "file_path": os.path.join("root", "opt", "bin", "e"),
                "content": "#!/usr/bin/python\nraise Exception()",
                "expected": "#!/usr/bin/env python\nraise Exception()",
            },
        ),
    ]

    def test_fix_shebang(self):
        for _, data in self.scenarios:
            os.makedirs(os.path.dirname(data["file_path"]), exist_ok=True)
            with open(data["file_path"], "w") as fd:
                fd.write(data["content"])

        BaseRepo.normalize("root")

        for _, data in self.scenarios:
            with open(data["file_path"], "r") as fd:
                self.expectThat(fd.read(), Equals(data["expected"]))


class RemoveUselessFilesTestCase(RepoBaseTestCase):
    def create(self, file_path):
        path = os.path.join("root", file_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        open(path, "w").close()

        return path

    def test_remove(self):
        paths = [
            self.create(p)
            for p in [
                os.path.join("usr", "lib", "python3.5", "sitecustomize.py"),
                os.path.join("usr", "lib", "python2.7", "sitecustomize.py"),
                os.path.join("usr", "lib", "python", "sitecustomize.py"),
            ]
        ]

        BaseRepo.normalize("root")

        for p in paths:
            self.expectThat(p, Not(FileExists()))

    def test_no_remove(self):
        path = self.create(os.path.join("opt", "python3.5", "sitecustomize.py"))

        BaseRepo.normalize("root")

        self.assertThat(path, FileExists())


class FixPkgConfigTestCase(RepoBaseTestCase):
    def test_fix_pkg_config(self):
        pc_file = os.path.join(self.tempdir, "granite.pc")

        with open(pc_file, "w") as f:
            f.write(
                dedent(
                    """\
                prefix=/usr
                exec_prefix=${prefix}
                libdir=${prefix}/lib
                includedir=${prefix}/include

                Name: granite
                Description: elementary\'s Application Framework
                Version: 0.4
                Libs: -L${libdir} -lgranite
                Cflags: -I${includedir}/granite
                Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
            """
                )
            )

        BaseRepo.normalize(self.tempdir)

        expected_pc_file_content = dedent(
            """\
            prefix={}/usr
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
                self.tempdir
            )
        )

        self.assertThat(pc_file, FileContains(expected_pc_file_content))

    def test_skip_directories_matching_pc(self):
        snap_pc_dir = os.path.join(self.tempdir, "snap.pc")

        os.mkdir(snap_pc_dir)

        # Verify the directory is not passed to fileinput, which would
        # try to file copy snap.pc to snap.pc.bak for the inplace replace.
        BaseRepo.normalize(self.tempdir)


class TestFixSymlinks(RepoBaseTestCase):
    def assert_fix(self, src, dst):
        os.makedirs("a")
        open("1", mode="w").close()

        os.symlink(src, dst)

        BaseRepo.normalize(self.tempdir)

        self.assertThat(os.readlink(dst), Equals(src))

    def test_rel_to_a(self):
        self.assert_fix("a", "rel-to-a")

    def test_abs_to_a(self):
        self.assert_fix("/a", "abs-to-a")


class FixSUIDTestCase(RepoBaseTestCase):
    def assert_mode(self, key, test_mod, expected_mod):
        file = os.path.join(self.tempdir, key)
        open(file, mode="w").close()
        os.chmod(file, test_mod)

        BaseRepo.normalize(self.tempdir)

        self.assertThat(stat.S_IMODE(os.stat(file).st_mode), Equals(expected_mod))

    def test_suid(self):
        self.assert_mode(key="suid_file", test_mod=0o4765, expected_mod=0o0765)

    def test_guid(self):
        self.assert_mode(key="guid_file", test_mod=0o2777, expected_mod=0o0777)

    def test_suid_guid(self):
        self.assert_mode(key="suid_guid_file", test_mod=0o6744, expected_mod=0o0744)

    def test_sticky_suid_guid(self):
        self.assert_mode(
            key="suid_guid_sticky_file", test_mod=0o7744, expected_mod=0o1744
        )


class TestPkgNameParts(unit.TestCase):
    def test_get_pkg_name_parts_name_only(self):
        name, version = get_pkg_name_parts("hello")
        self.assertThat(name, Equals("hello"))
        self.assertThat(version, Equals(None))

    def test_get_pkg_name_parts_all(self):
        name, version = get_pkg_name_parts("hello:i386=2.10-1")
        self.assertThat(name, Equals("hello:i386"))
        self.assertThat(version, Equals("2.10-1"))

    def test_get_pkg_name_parts_no_arch(self):
        name, version = get_pkg_name_parts("hello=2.10-1")
        self.assertThat(name, Equals("hello"))
        self.assertThat(version, Equals("2.10-1"))
