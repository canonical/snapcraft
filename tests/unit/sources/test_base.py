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
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal.sources import _base
from tests import unit


class TestFileBase(unit.TestCase):
    def get_mock_file_base(self, source, dir):
        file_src = _base.FileBase(source, dir)
        setattr(file_src, "provision", mock.Mock())
        return file_src

    @mock.patch("snapcraft.internal.sources._base.FileBase.download")
    def test_pull_url(self, mock_download):
        mock_download.return_value = "dir"
        file_src = self.get_mock_file_base("http://snapcraft.io/snapcraft.yaml", "dir")
        file_src.pull()

        mock_download.assert_called_once_with()
        file_src.provision.assert_called_once_with(
            file_src.source_dir, src="dir", clean_target=False
        )

    @mock.patch("shutil.copy2")
    def test_pull_copy(self, mock_shutil_copy2):
        file_src = self.get_mock_file_base("snapcraft.yaml", "dir")
        file_src.pull()

        expected = os.path.join(file_src.source_dir, "snapcraft.yaml")
        mock_shutil_copy2.assert_called_once_with(file_src.source, expected)
        file_src.provision.assert_called_once_with(
            file_src.source_dir, src=expected, clean_target=False
        )

    @mock.patch("snapcraft.internal.sources._base.requests")
    @mock.patch("snapcraft.internal.sources._base.download_requests_stream")
    @mock.patch("snapcraft.internal.sources._base.download_urllib_source")
    def test_download_file_destination(self, dus, drs, req):
        file_src = self.get_mock_file_base("http://snapcraft.io/snapcraft.yaml", "dir")
        self.assertFalse(hasattr(file_src, "file"))

        file_src.pull()

        self.assertThat(
            file_src.file,
            Equals(
                os.path.join(file_src.source_dir, os.path.basename(file_src.source))
            ),
        )

    @mock.patch("snapcraft.internal.sources._base.download_requests_stream")
    @mock.patch("snapcraft.internal.sources._base.requests")
    def test_download_http(self, mock_requests, mock_download):
        file_src = self.get_mock_file_base("http://snapcraft.io/snapcraft.yaml", "dir")

        mock_request = mock.Mock()
        mock_requests.get.return_value = mock_request

        file_src.pull()

        mock_requests.get.assert_called_once_with(
            file_src.source, stream=True, allow_redirects=True
        )
        mock_request.raise_for_status.assert_called_once_with()
        mock_download.assert_called_once_with(mock_request, file_src.file)

    @mock.patch("snapcraft.internal.sources._base.download_urllib_source")
    def test_download_ftp(self, mock_download):
        file_src = self.get_mock_file_base("ftp://snapcraft.io/snapcraft.yaml", "dir")

        file_src.pull()

        mock_download.assert_called_once_with(file_src.source, file_src.file)

    @mock.patch("snapcraft.internal.indicators.urlretrieve")
    def test_download_ftp_url_opener(self, mock_urlretrieve):
        file_src = self.get_mock_file_base("ftp://snapcraft.io/snapcraft.yaml", "dir")

        file_src.pull()

        self.assertThat(mock_urlretrieve.call_count, Equals(1))
        self.assertThat(mock_urlretrieve.call_args[0][0], Equals(file_src.source))
        self.assertThat(mock_urlretrieve.call_args[0][1], Equals(file_src.file))
