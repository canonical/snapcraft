# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2022 Canonical Ltd
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
from pathlib import Path
from textwrap import dedent

import pytest
from testtools.matchers import Equals, FileContains, FileExists, Not

from snapcraft_legacy.internal.repo._base import (
    BaseRepo,
    fix_pkg_config,
    get_pkg_name_parts,
)
from tests.legacy import unit

from . import RepoBaseTestCase


@pytest.fixture()
def pkg_config_file():
    """Fixture for writing a pkg-config (.pc) files."""

    def _pkg_config_file(filename: Path, prefix: str) -> None:
        """Writes a pkg-config file.

        :param filename: filename of pkg-config file
        :param prefix: value of prefix parameter
        """
        with open(filename, "w", encoding="utf-8") as file:
            file.write(
                dedent(
                    f"""\
                    prefix={prefix}
                    exec_prefix=${{prefix}}
                    libdir=${{prefix}}/lib
                    includedir=${{prefix}}/include

                    Name: granite
                    Description: elementary\'s Application Framework
                    Version: 0.4
                    Libs: -L${{libdir}} -lgranite
                    Cflags: -I${{includedir}}/granite
                    Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
                    """
                )
            )

    yield _pkg_config_file


@pytest.fixture()
def expected_pkg_config_content():
    """Returns a string containing the expected content of the pkg-config fixture."""

    def _expected_pkg_config_content(prefix: str) -> str:
        """Returns the expected contents of a pkg-config file.

        :param prefix: value of the prefix parameter
        """
        return dedent(
            f"""\
            prefix={prefix}
            exec_prefix=${{prefix}}
            libdir=${{prefix}}/lib
            includedir=${{prefix}}/include

            Name: granite
            Description: elementary's Application Framework
            Version: 0.4
            Libs: -L${{libdir}} -lgranite
            Cflags: -I${{includedir}}/granite
            Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
            """
        )

    yield _expected_pkg_config_content


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


@pytest.mark.parametrize(
    "prefix,fixed_prefix",
    [
        # possible prefixes from snaps built via launchpad
        ("/build/mir-core20/stage", ""),
        ("/build/mir-core20/stage/usr", "/usr"),
        ("/build/stage/stage", ""),
        ("/build/stage/stage/usr/stage", "/usr/stage"),
        ("/build/my-stage-snap/stage", ""),
        ("/build/my-stage-snap/stage/usr/stage", "/usr/stage"),
        # possible prefixes from snaps built via a provider
        ("/root/stage", ""),
        ("/root/stage/usr", "/usr"),
        ("/root/stage/usr/stage", "/usr/stage"),
    ],
)
def test_fix_pkg_config_trim_prefix_from_snap(
    tmpdir,
    prefix,
    fixed_prefix,
    pkg_config_file,
    expected_pkg_config_content,
):
    """Verify prefixes from snaps are trimmed."""
    pc_file = tmpdir / "my-file.pc"
    pkg_config_file(pc_file, prefix)

    fix_pkg_config(tmpdir, pc_file)

    assert pc_file.read_text(encoding="utf-8") == expected_pkg_config_content(
        f"{tmpdir}{fixed_prefix}"
    )


@pytest.mark.parametrize(
    "prefix",
    [
        "",
        "/",
        "/usr",
        "/build/test/test/stage",
        "/root/test/stage",
        "/test/path/stage",
        "/test/path/stage/usr",
        "/test/path/stage/usr/stage",
    ],
)
def test_fix_pkg_config_no_trim(
    tmpdir,
    prefix,
    pkg_config_file,
    expected_pkg_config_content,
):
    """Verify valid prefixes are not trimmed."""
    pc_file = tmpdir / "my-file.pc"
    pkg_config_file(pc_file, prefix)

    fix_pkg_config(tmpdir, pc_file)

    assert pc_file.read_text(encoding="utf-8") == expected_pkg_config_content(
        f"{tmpdir}{prefix}"
    )


@pytest.mark.parametrize(
    "prefix,prefix_trim,fixed_prefix",
    [
        ("/test/build/dir/stage", "/test/build/dir/stage", ""),
        ("/test/build/dir/stage/usr", "/test/build/dir/stage", "/usr"),
        ("/fake/dir/1", "/fake/dir/2", "/fake/dir/1"),
    ],
)
def test_fix_pkg_config_trim_prefix(
    tmpdir,
    prefix,
    prefix_trim,
    fixed_prefix,
    pkg_config_file,
    expected_pkg_config_content,
):
    """Verify prefixes from the `prefix_trim` argument are trimmed."""
    pc_file = tmpdir / "my-file.pc"
    pkg_config_file(pc_file, prefix)

    fix_pkg_config(tmpdir, pc_file, prefix_trim)

    assert pc_file.read_text(encoding="utf-8") == expected_pkg_config_content(
        f"{tmpdir}{fixed_prefix}"
    )


def test_fix_pkg_config_with_pcfiledir(
    tmpdir,
    pkg_config_file,
    expected_pkg_config_content,
):
    """Verify prefixes that begin with ${pcfiledir} aren't modified."""
    pc_file = tmpdir / "my-file.pc"
    pkg_config_file(pc_file, "${pcfiledir}/../../..")

    fix_pkg_config(tmpdir, pc_file)

    assert pc_file.read_text(encoding="utf-8") == expected_pkg_config_content(
        "${pcfiledir}/../../.."
    )


def test_normalize_fix_pkg_config(tmpdir, pkg_config_file, expected_pkg_config_content):
    """Verify normalization fixes pkg-config files."""
    pc_file = tmpdir / "my-file.pc"
    pkg_config_file(pc_file, "/root/stage/usr")
    BaseRepo.normalize(tmpdir)

    assert pc_file.read_text(encoding="utf-8") == expected_pkg_config_content(
        f"{tmpdir}/usr"
    )


def test_fix_pkg_config_is_dir(tmpdir):
    """Verify directories ending in .pc do not raise an error."""
    pc_file = tmpdir / "granite.pc"
    pc_file.mkdir()

    BaseRepo.normalize(tmpdir)


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
