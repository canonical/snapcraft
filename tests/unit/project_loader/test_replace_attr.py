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

from testtools.matchers import Equals

from snapcraft.internal import project_loader

from tests import unit


class VariableReplacementsTest(unit.TestCase):
    def test_string_replacements(self):
        replacements = (
            ("no replacement", "snapcraft_stage/usr/bin", "snapcraft_stage/usr/bin"),
            (
                "replaced start",
                "$SNAPCRAFT_STAGE/usr/bin",
                "{}/usr/bin".format(self.stage_dir),
            ),
            (
                "replaced between",
                "--with-swig $SNAPCRAFT_STAGE/usr/swig",
                "--with-swig {}/usr/swig".format(self.stage_dir),
            ),
            (
                "project replacement",
                "$SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION",
                "project_name-version",
            ),
            (
                "multiple replacement",
                "$SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_NAME",
                "project_name-project_name",
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(
                project_loader.replace_attr(
                    subject,
                    {
                        "$SNAPCRAFT_PROJECT_NAME": "project_name",
                        "$SNAPCRAFT_PROJECT_VERSION": "version",
                        "$SNAPCRAFT_STAGE": self.stage_dir,
                    },
                ),
                Equals(expected),
            )

    def test_lists_with_string_replacements(self):
        replacements = (
            (
                "no replacement",
                ["snapcraft_stage/usr/bin", "/usr/bin"],
                ["snapcraft_stage/usr/bin", "/usr/bin"],
            ),
            (
                "replaced start",
                ["$SNAPCRAFT_STAGE/usr/bin", "/usr/bin"],
                ["{}/usr/bin".format(self.stage_dir), "/usr/bin"],
            ),
            (
                "replaced between",
                ["--without-python", "--with-swig $SNAPCRAFT_STAGE/usr/swig"],
                ["--without-python", "--with-swig {}/usr/swig".format(self.stage_dir)],
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(
                project_loader.replace_attr(
                    subject,
                    {
                        "$SNAPCRAFT_PROJECT_NAME": "project_name",
                        "$SNAPCRAFT_PROJECT_VERSION": "version",
                        "$SNAPCRAFT_STAGE": self.stage_dir,
                    },
                ),
                Equals(expected),
            )

    def test_tuples_with_string_replacements(self):
        replacements = (
            (
                "no replacement",
                ("snapcraft_stage/usr/bin", "/usr/bin"),
                ["snapcraft_stage/usr/bin", "/usr/bin"],
            ),
            (
                "replaced start",
                ("$SNAPCRAFT_STAGE/usr/bin", "/usr/bin"),
                ["{}/usr/bin".format(self.stage_dir), "/usr/bin"],
            ),
            (
                "replaced between",
                ("--without-python", "--with-swig $SNAPCRAFT_STAGE/usr/swig"),
                ["--without-python", "--with-swig {}/usr/swig".format(self.stage_dir)],
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(
                project_loader.replace_attr(
                    subject,
                    {
                        "$SNAPCRAFT_PROJECT_NAME": "project_name",
                        "$SNAPCRAFT_PROJECT_VERSION": "version",
                        "$SNAPCRAFT_STAGE": self.stage_dir,
                    },
                ),
                Equals(expected),
            )

    def test_dict_with_string_replacements(self):
        replacements = (
            (
                "no replacement",
                {"1": "snapcraft_stage/usr/bin", "2": "/usr/bin"},
                {"1": "snapcraft_stage/usr/bin", "2": "/usr/bin"},
            ),
            (
                "replaced start",
                {"1": "$SNAPCRAFT_STAGE/usr/bin", "2": "/usr/bin"},
                {"1": "{}/usr/bin".format(self.stage_dir), "2": "/usr/bin"},
            ),
            (
                "replaced between",
                {"1": "--without-python", "2": "--with-swig $SNAPCRAFT_STAGE/usr/swig"},
                {
                    "1": "--without-python",
                    "2": "--with-swig {}/usr/swig".format(self.stage_dir),
                },
            ),
            (
                "replace keys as well",
                {"$SNAPCRAFT_STAGE": "--with-swig $SNAPCRAFT_STAGE"},
                {self.stage_dir: "--with-swig {}".format(self.stage_dir)},
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(
                project_loader.replace_attr(
                    subject,
                    {
                        "$SNAPCRAFT_PROJECT_NAME": "project_name",
                        "$SNAPCRAFT_PROJECT_VERSION": "version",
                        "$SNAPCRAFT_STAGE": self.stage_dir,
                    },
                ),
                Equals(expected),
            )

    def test_string_replacement_with_complex_data(self):
        subject = {
            "filesets": {
                "files": [
                    "somefile",
                    "$SNAPCRAFT_STAGE/file1",
                    "SNAPCRAFT_STAGE/really",
                ]
            },
            "configflags": ["--with-python", "--with-swig $SNAPCRAFT_STAGE/swig"],
        }

        expected = {
            "filesets": {
                "files": [
                    "somefile",
                    "{}/file1".format(self.stage_dir),
                    "SNAPCRAFT_STAGE/really",
                ]
            },
            "configflags": [
                "--with-python",
                "--with-swig {}/swig".format(self.stage_dir),
            ],
        }

        self.assertThat(
            project_loader.replace_attr(
                subject,
                {
                    "$SNAPCRAFT_PROJECT_NAME": "project_name",
                    "$SNAPCRAFT_PROJECT_VERSION": "version",
                    "$SNAPCRAFT_STAGE": self.stage_dir,
                },
            ),
            Equals(expected),
        )
