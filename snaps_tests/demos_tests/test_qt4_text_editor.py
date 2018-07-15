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

import snaps_tests


class Qt4TextEditorTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = "qt4-text-editor"

    def test_text_editor(self):
        snap_path = self.build_snap(self.snap_content_dir)
        self.install_snap(snap_path, "text-editor", "1.0")

        # TODO: Need xvfb in order to actually run
