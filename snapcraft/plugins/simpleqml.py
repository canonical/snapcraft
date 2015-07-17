# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import apt
import os
import snapcraft.common
import snapcraft.plugins.ubuntu
import subprocess
import sys


class SimpleQtQuickPlugin(snapcraft.plugins.ubuntu.UbuntuPlugin):

    def __init__(self, name, options):
        options.package = None
        super().__init__(name, options)
        if options.qml is not None:
            self.qml = options.qml
        else:
            self.qml = name
        if not os.path.isfile(self.qml):
            snapcraft.common.log("qml file %r does not exist" % (self.qml,))
            sys.exit(1)
        self.includedPackages = [
            "qmlscene",
            "qtdeclarative5-qtmir-plugin",
            "mir-graphics-drivers-desktop",
            "qtubuntu-desktop",
            # if there's a metapackage for these, please swap it in here:
            "qml-module-qt-labs-folderlistmodel",
            "qml-module-qt-labs-settings",
            "qml-module-qt-websockets",
            "qml-module-qtfeedback",
            "qml-module-qtgraphicaleffects",
            "qml-module-qtlocation",
            "qml-module-qtmultimedia",
            "qml-module-qtorganizer",
            "qml-module-qtpositioning",
            "qml-module-qtqml-models2",
            "qml-module-qtqml-statemachine",
            "qml-module-qtquick-controls",
            "qml-module-qtquick-dialogs",
            "qml-module-qtquick-layouts",
            "qml-module-qtquick-localstorage",
            "qml-module-qtquick-particles2",
            "qml-module-qtquick-privatewidgets",
            "qml-module-qtquick-window2",
            "qml-module-qtquick-xmllistmodel",
            "qml-module-qtquick2",
            "qml-module-qtsensors",
            "qml-module-qtsysteminfo",
            "qml-module-qttest",
            "qml-module-qtwebkit",
            "qml-module-ubuntu-connectivity",
            "qml-module-ubuntu-onlineaccounts",
            "qml-module-ubuntu-onlineaccounts-client",
        ]



    def snapFiles(self):
        return (['usr/lib/x86_64-linux-gnu'],
                [])

