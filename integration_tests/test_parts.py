# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import integration_tests


class PartsTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.parts_dir = os.path.join('data', 'snapcraft')
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')
        self.headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')

    def test_update(self):
        self.run_snapcraft('update')

        self.assertTrue(os.path.exists(self.parts_yaml))
        self.assertTrue(os.path.exists(self.headers_yaml))

    def test_curl_exists_and_properly_defined(self):
        """Curl is used in most of the demos so we test for its existence."""
        self.run_snapcraft('update')
        output = self.run_snapcraft(['define', 'curl'])

        expected = (
            "Maintainer: 'Sergio Schvezov <sergio.schvezov@ubuntu.com>'\n"
            "Description: A tool and a library (usable from many languages) "
            "for client side URL transfers, supporting FTP, FTPS, HTTP, "
            "HTTPS, TELNET, DICT, FILE and LDAP.\n\n"
            "curl:\n"
            "  plugin: autotools\n"
            "  source: http://curl.haxx.se/download/curl-7.44.0.tar.bz2\n"
            "  source-type: tar\n"
            "  configflags:\n"
            "  - --enable-static\n"
            "  - --enable-shared\n"
            "  - --disable-manual\n"
            "  snap:\n"
            "  - -bin\n"
            "  - -lib/*.a\n"
            "  - -lib/pkgconfig\n"
            "  - -lib/*.la\n"
            "  - -include\n"
            "  - -share\n")

        self.assertEqual(expected, output)
