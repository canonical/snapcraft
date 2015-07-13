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
import subprocess
import sys


class UbuntuPlugin(snapcraft.BasePlugin):

    def __init__(self, name, config, options):
        super().__init__(name, config, options)
        self.downloadablePackages = []
        self.includedPackages = []
        if options.package:
            self.includedPackages.append(options.package)
        else:
            # User didn't specify a package, use the part name
            if name == 'ubuntu':
                snapcraft.common.log("Part %s needs either a package option or a name" % name)
                sys.exit(1)
            self.includedPackages.append(name)

    def pull(self):
        self.downloadablePackages = self.get_all_dep_packages(self.includedPackages)
        return self.download_debs(self.downloadablePackages)

    def build(self):
        if not self.downloadablePackages:
            self.downloadablePackages = self.get_all_dep_packages(self.includedPackages)
        return self.unpack_debs(self.downloadablePackages, self.installdir)

    def snap_files(self):
        return (['*'], ['/usr/include', '/lib/*/*.a', '/usr/lib/*/*.a'])

    def get_all_dep_packages(self, packages):
        cache = apt.Cache()
        alldeps = set()
        manifestdeps = set()
        skipped = set()

        with open(os.path.abspath(os.path.join(__file__, '..', 'manifest.txt'))) as f:
            for line in f:
                pkg = line.strip()
                if pkg in cache:
                    manifestdeps.add(pkg)

        def add_deps(pkgs):
            for p in pkgs:
                if p in alldeps:
                    continue
                if p in manifestdeps and p not in packages:
                    skipped.add(p)
                    continue
                try:
                    deps = set()
                    candidatePkg = cache[p].candidate
                    deps = candidatePkg.dependencies + candidatePkg.recommends
                    alldeps.add(p)
                    add_deps([x[0].name for x in deps])
                except:
                    pass

        add_deps(packages)

        exit = False
        for p in packages:
            if p not in alldeps:
                exit = True
                snapcraft.common.log("Package %s not recognized" % p, file=sys.stderr)
        if exit:
            sys.exit(1)

        return sorted(alldeps)

    def download_debs(self, pkgs, debdir=None):
        debdir = debdir or self.builddir
        if pkgs:
            return self.run(['dget'] + pkgs, cwd=debdir, stdout=subprocess.DEVNULL)
        else:
            return True

    def unpack_debs(self, pkgs, targetDir, debdir=None):
        debdir = debdir or self.builddir
        for p in pkgs:
            if not self.run(['dpkg-deb', '--extract', p + '_*.deb', targetDir], cwd=debdir):
                return False
        return True
