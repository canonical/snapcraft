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

import hashlib
import os
import sys
import xdg

import yaml
from testtools.matchers import Contains, Equals

from tests import integration


class PartsTestCase(integration.TestCase):
    def _parts_dir(self):
        parts_uri = "https://parts.snapcraft.io/v1/parts.yaml"
        return os.path.join(
            xdg.BaseDirectory.save_data_path("snapcraft"),
            hashlib.sha384(parts_uri.encode(sys.getfilesystemencoding())).hexdigest(),
        )

    def setUp(self):
        super().setUp()

        self.parts_dir = self._parts_dir()
        self.parts_yaml = os.path.join(self.parts_dir, "parts.yaml")
        self.headers_yaml = os.path.join(self.parts_dir, "headers.yaml")

    def test_update(self):
        self.run_snapcraft("update")

        self.assertTrue(os.path.exists(self.parts_yaml))
        self.assertTrue(os.path.exists(self.headers_yaml))

    def test_curl_exists_and_properly_defined(self):
        """Curl is used in most of the demos so we test for its existence."""
        self.run_snapcraft("update")
        output = self.run_snapcraft(["define", "curl"])

        expected_prefix = (
            "Maintainer: 'Sergio Schvezov <sergio.schvezov@ubuntu.com>'\n"
            "Description: A tool and a library (usable from many languages) "
            "for client side URL transfers, supporting FTP, FTPS, HTTP, "
            "HTTPS, TELNET, DICT, FILE and LDAP.\n\n"
        )
        self.assertThat(output, Contains(expected_prefix))
        idx = output.index(expected_prefix) + len(expected_prefix)
        part = yaml.safe_load(output[idx:])
        expected_part = {
            "curl": {
                "plugin": "autotools",
                "source": "http://curl.haxx.se/download/curl-7.44.0.tar.bz2",
                "source-type": "tar",
                "configflags": [
                    "--enable-static",
                    "--enable-shared",
                    "--disable-manual",
                ],
                "snap": [
                    "-bin",
                    "-lib/*.a",
                    "-lib/pkgconfig",
                    "-lib/*.la",
                    "-include",
                    "-share",
                ],
            }
        }
        self.assertThat(part, Equals(expected_part))


class PartsWithFilesetsTestCase(integration.TestCase):
    def test_part_with_fileset(self):
        self.run_snapcraft("update")

        output = self.run_snapcraft(["define", "simple-make-filesets"])

        expected_prefix = (
            "Maintainer: 'Jonathan Cave <jonathan.cave@canonical.com>'\n"
            "Description: The filesets test from the integration test suite."
            "\n\n"
        )
        self.assertThat(output, Contains(expected_prefix))
        idx = output.index(expected_prefix) + len(expected_prefix)
        part = yaml.safe_load(output[idx:])
        expected_part = {
            "simple-make-filesets": {
                "plugin": "make",
                "filesets": {"files": ["share/file1", "share/file2"]},
                "stage": ["$files", "new/dir1", "new/dir2"],
                "snap": ["-new/dir1"],
                "organize": {
                    "file1": "share/file1",
                    "file2": "share/file2",
                    "dir1": "new/dir1",
                    "dir2": "new/dir2",
                },
                "source": "https://github.com/jocave/simple-make-filesets.git",
            }
        }
        self.assertThat(part, Equals(expected_part))

        self.run_snapcraft("snap", "wiki-filesets")
