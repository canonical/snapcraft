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

import os
import tarfile
import fixtures
from unittest import mock

import requests
from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


class TestTar(unit.FakeFileHTTPServerBasedTestCase):

    scenarios = [("TERM=dumb", dict(term="dumb")), ("TERM=vt100", dict(term="vt100"))]

    def setUp(self):
        self.useFixture(fixtures.EnvironmentVariable("TERM", self.term))
        super().setUp()

    @mock.patch("snapcraft.sources.Tar.provision")
    def test_pull_tarball_must_download_to_sourcedir(self, mock_prov):
        plugin_name = "test_plugin"
        dest_dir = os.path.join("parts", plugin_name, "src")
        os.makedirs(dest_dir)
        tar_file_name = "test.tar"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=tar_file_name
        )
        tar_source = sources.Tar(source, dest_dir)

        tar_source.pull()

        source_file = os.path.join(dest_dir, tar_file_name)
        mock_prov.assert_called_once_with(dest_dir, src=source_file, clean_target=False)
        with open(os.path.join(dest_dir, tar_file_name), "r") as tar_file:
            self.assertThat(tar_file.read(), Equals("Test fake file"))

    @mock.patch("snapcraft.sources.Tar.provision")
    def test_pull_twice_downloads_once(self, mock_prov):
        """If a source checksum is defined, the cache should be tried first."""
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name="test.tar"
        )
        expected_checksum = (
            "sha384/d9da1f5d54432edc8963cd817ceced83f7c6d61d3"
            "50ad76d1c2f50c4935d11d50211945ca0ecb980c04c98099"
            "085b0c3"
        )
        tar_source = sources.Tar(source, self.path, source_checksum=expected_checksum)

        tar_source.pull()
        with mock.patch(
            "requests.get", new=mock.Mock(wraps=requests.get)
        ) as download_spy:
            tar_source.pull()
            self.assertThat(download_spy.call_count, Equals(0))

    def test_strip_common_prefix(self):
        # Create tar file for testing
        os.makedirs(os.path.join("src", "test_prefix"))
        file_to_tar = os.path.join("src", "test_prefix", "test.txt")
        open(file_to_tar, "w").close()
        tar = tarfile.open(os.path.join("src", "test.tar"), "w")
        tar.add(file_to_tar)
        tar.close()

        tar_source = sources.Tar(os.path.join("src", "test.tar"), "dst")
        os.mkdir("dst")
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join("dst", "test.txt")))

    def test_strip_common_prefix_symlink(self):
        # Create tar file for testing
        os.makedirs(os.path.join("src", "test_prefix"))
        file_to_tar = os.path.join("src", "test_prefix", "test.txt")
        open(file_to_tar, "w").close()

        file_to_link = os.path.join("src", "test_prefix", "link.txt")
        os.symlink("./test.txt", file_to_link)
        self.assertTrue(os.path.islink(file_to_link))

        def check_for_symlink(tarinfo):
            self.assertTrue(tarinfo.issym())
            self.assertThat(file_to_link, Equals(tarinfo.name))
            self.assertThat(
                file_to_tar,
                Equals(
                    os.path.normpath(
                        os.path.join(os.path.dirname(file_to_tar), tarinfo.linkname)
                    )
                ),
            )
            return tarinfo

        tar = tarfile.open(os.path.join("src", "test.tar"), "w")
        tar.add(file_to_tar)
        tar.add(file_to_link, filter=check_for_symlink)
        tar.close()

        tar_source = sources.Tar(os.path.join("src", "test.tar"), "dst")
        os.mkdir("dst")
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join("dst", "test.txt")))
        self.assertTrue(os.path.exists(os.path.join("dst", "link.txt")))

    def test_strip_common_prefix_hardlink(self):
        # Create tar file for testing
        os.makedirs(os.path.join("src", "test_prefix"))
        file_to_tar = os.path.join("src", "test_prefix", "test.txt")
        open(file_to_tar, "w").close()

        file_to_link = os.path.join("src", "test_prefix", "link.txt")
        os.link(file_to_tar, file_to_link)
        self.assertTrue(os.path.exists(file_to_link))

        def check_for_hardlink(tarinfo):
            self.assertTrue(tarinfo.islnk())
            self.assertFalse(tarinfo.issym())
            self.assertThat(file_to_link, Equals(tarinfo.name))
            self.assertThat(file_to_tar, Equals(tarinfo.linkname))
            return tarinfo

        tar = tarfile.open(os.path.join("src", "test.tar"), "w")
        tar.add(file_to_tar)
        tar.add(file_to_link, filter=check_for_hardlink)
        tar.close()

        tar_source = sources.Tar(os.path.join("src", "test.tar"), "dst")
        os.mkdir("dst")
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join("dst", "test.txt")))
        self.assertTrue(os.path.exists(os.path.join("dst", "link.txt")))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["tar"] is sources.Tar)
