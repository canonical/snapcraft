#!/bin/bash
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

apt-key adv --keyserver keyserver.ubuntu.com --recv-key 78E1918602959B9C59103100F1831DDAFC42E99D
set -x; REL=$(sed -rn "/^(deb|deb-src) .*(ubuntu.com|ftpmaster)/ { s/^[^ ]+ +(\[.*\] *)?[^ ]* +([^ -]+) +.*$/\\2/p; q }" /etc/apt/sources.list)
echo "deb http://ppa.launchpad.net/snappy-dev/snapcraft-daily/ubuntu $REL main\ndeb-src http://ppa.launchpad.net/snappy-dev/snapcraft-daily/ubuntu $REL main" > /etc/apt/sources.list.d/autopkgtest-snapcraft.list
