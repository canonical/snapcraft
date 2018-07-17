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

from snapcraft.internal import errors
from snapcraft.internal.repo import check_for_command
from snapcraft.internal.repo import BaseRepo
from tests import unit
from . import RepoBaseTestCase


class CommandCheckTestCase(unit.TestCase):
    def test_check_for_command_not_installed(self):
        self.assertRaises(
            errors.MissingCommandError, check_for_command, "missing-command"
        )

    def test_check_for_command_installed(self):
        check_for_command("sh")


class FixXmlToolsTestCase(RepoBaseTestCase):

    scenarios = [
        (
            "xml2-config only should fix",
            {
                "files": [
                    {
                        "path": os.path.join("root", "usr", "bin", "xml2-config"),
                        "content": "prefix=/usr/foo",
                        "expected": "prefix=root/usr/foo",
                    }
                ]
            },
        ),
        (
            "xml2-config only should not fix",
            {
                "files": [
                    {
                        "path": os.path.join("root", "usr", "bin", "xml2-config"),
                        "content": "prefix=/foo",
                        "expected": "prefix=/foo",
                    }
                ]
            },
        ),
        (
            "xslt-config only should fix",
            {
                "files": [
                    {
                        "path": os.path.join("root", "usr", "bin", "xslt-config"),
                        "content": "prefix=/usr/foo",
                        "expected": "prefix=root/usr/foo",
                    }
                ]
            },
        ),
        (
            "xslt-config only should not fix",
            {
                "files": [
                    {
                        "path": os.path.join("root", "usr", "bin", "xslt-config"),
                        "content": "prefix=/foo",
                        "expected": "prefix=/foo",
                    }
                ]
            },
        ),
        (
            "xml2-config and xslt-config",
            {
                "files": [
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
            },
        ),
        (
            "xml2-config and xslt-config should not fix",
            {
                "files": [
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
            },
        ),
    ]

    def test_fix_xml_tools(self):
        for test_file in self.files:
            path = test_file["path"]
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                f.write(test_file["content"])

        BaseRepo("root").normalize("root")

        for test_file in self.files:
            self.assertThat(test_file["path"], FileContains(test_file["expected"]))


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
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        with open(self.file_path, "w") as fd:
            fd.write(self.content)

        BaseRepo("root").normalize("root")

        with open(self.file_path, "r") as fd:
            self.assertThat(fd.read(), Equals(self.expected))


class RemoveUselessFilesTestCase(RepoBaseTestCase):

    scenarios = [
        (
            "python3 sitecustomize",
            {
                "file_path": os.path.join(
                    "usr", "lib", "python3.5", "sitecustomize.py"
                ),
                "matcher": Not(FileExists()),
            },
        ),
        (
            "python2 sitecustomize",
            {
                "file_path": os.path.join(
                    "usr", "lib", "python2.7", "sitecustomize.py"
                ),
                "matcher": Not(FileExists()),
            },
        ),
        (
            "unversioned sitecustomize",
            {
                "file_path": os.path.join("usr", "lib", "python", "sitecustomize.py"),
                "matcher": Not(FileExists()),
            },
        ),
        (
            "random sitecustomize",
            {
                "file_path": os.path.join("opt", "python3.5", "sitecustomize.py"),
                "matcher": FileExists(),
            },
        ),
    ]

    def test_remove_useless_files(self):
        path = os.path.join("root", self.file_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        open(path, "w").close()

        BaseRepo("root").normalize("root")

        self.assertThat(path, self.matcher)


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

        BaseRepo(self.tempdir).normalize(self.tempdir)

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


class FixSymlinksTestCase(RepoBaseTestCase):

    scenarios = [
        ("rel-to-a", {"src": "a", "dst": "rel-to-a"}),
        ("abs-to-a", {"src": "/a", "dst": "abs-to-a"}),
        ("abs-to-b", {"src": "/b", "dst": "abs-to-b"}),
        ("rel-to-1", {"src": "1", "dst": "rel-to-1"}),
        ("abs-to-1", {"src": "/1", "dst": "abs-to-1"}),
    ]

    def setUp(self):
        super().setUp()
        os.makedirs("a")
        open("1", mode="w").close()

    def test_fix_symlinks(self):
        os.symlink(self.src, self.dst)

        BaseRepo(self.tempdir).normalize(self.tempdir)

        self.assertThat(os.readlink(self.dst), Equals(self.src))


class FixSUIDTestCase(RepoBaseTestCase):

    scenarios = [
        ("suid_file", dict(key="suid_file", test_mod=0o4765, expected_mod=0o0765)),
        ("guid_file", dict(key="guid_file", test_mod=0o2777, expected_mod=0o0777)),
        (
            "suid_guid_file",
            dict(key="suid_guid_file", test_mod=0o6744, expected_mod=0o0744),
        ),
        (
            "suid_guid_sticky_file",
            dict(key="suid_guid_sticky_file", test_mod=0o7744, expected_mod=0o1744),
        ),
    ]

    def test_fix_suid(self):
        file = os.path.join(self.tempdir, self.key)
        open(file, mode="w").close()
        os.chmod(file, self.test_mod)

        BaseRepo(self.tempdir).normalize(self.tempdir)

        self.assertThat(stat.S_IMODE(os.stat(file).st_mode), Equals(self.expected_mod))
