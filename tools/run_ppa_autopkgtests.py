#!/usr/bin/env python
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import subprocess
import tempfile

from launchpadlib.launchpad import Launchpad


ACTIVE_DISTROS = ("xenial", "artful", "bionic")
ACTIVE_ARCHITECTURES = ("amd64", "i386", "armhf", "arm64")


def main():
    try:
        cookie_file_path = save_cookie()
        for distro, architecture, version in snapcraft_ppa_packages():
            request_autopkgtest_execution(
                cookie_file_path, distro, architecture, version
            )
    finally:
        os.remove(cookie_file_path)


def save_cookie():
    cookie_file = tempfile.NamedTemporaryFile(delete=False)
    cookie_file.write(
        "autopkgtest.ubuntu.com\tTRUE\t/\tTRUE\t0\tsession\t{}".format(
            os.environ.get("SNAPCRAFT_AUTOPKGTEST_COOKIE")
        )
    )
    cookie_file.close()
    return cookie_file.name


def snapcraft_ppa_packages():
    launchpad = Launchpad.login_anonymously("snappy-m-o", "production")
    ubuntu = launchpad.distributions["ubuntu"]
    snapcraft_daily_ppa = launchpad.people["snappy-dev"].getPPAByName(
        name="snapcraft-daily"
    )

    for distro in ACTIVE_DISTROS:
        for architecture in ACTIVE_ARCHITECTURES:
            distro_arch = ubuntu.getSeries(name_or_version=distro).getDistroArchSeries(
                archtag=architecture
            )
            for package in snapcraft_daily_ppa.getPublishedBinaries(
                status="Published",
                binary_name="snapcraft",
                exact_match=True,
                distro_arch_series=distro_arch,
            ):
                yield distro, architecture, str(package.binary_package_version)


def request_autopkgtest_execution(cookie_path, distro, architecture, version):
    output = subprocess.check_output(
        [
            "wget",
            "-O-",
            "--load-cookies",
            cookie_path,
            "https://autopkgtest.ubuntu.com/request.cgi?release={distro}&"
            "arch={architecture}&package=snapcraft&"
            "ppa=snappy-dev/snapcraft-daily&"
            "trigger=snapcraft/{version}".format(
                distro=distro, architecture=architecture, version=version
            ),
        ]
    )
    if "Test request submitted" not in output:
        exit("Failed to request the autopkgtest")


if __name__ == "__main__":
    main()
