# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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
import logging
import shutil

import snapcraft
import snapcraft.sources

logger = logging.getLogger(__name__)


class TarContentPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        return {"properties": {"destination": {"type": "string"}}}

    @classmethod
    def get_build_properties(cls):
        return ["destination"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        logger.warning(
            "DEPRECATED: The 'tar-content' plugin's functionality "
            "has been replaced by the 'dump' plugin, and it will "
            "soon be removed."
        )

        if self.options.destination and os.path.isabs(self.options.destination):
            raise ValueError(
                "path {!r} must be relative".format(self.options.destination)
            )

    def enable_cross_compilation(self):
        pass

    def build(self):
        super().build()

        installdir = self.installdir
        if self.options.destination:
            installdir = os.path.join(self.installdir, self.options.destination)

        if os.path.exists(installdir):
            shutil.rmtree(installdir)

        shutil.copytree(self.builddir, installdir)
