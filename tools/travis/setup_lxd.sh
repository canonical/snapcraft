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

# Set up LXD in travis.

set -ev

sudo apt-get update
sudo apt-get install --yes snapd
sudo snap install lxd
# Wait while LXD first generates its keys. In a low entropy environment this
# can take a while.
sleep 10
sudo /snap/bin/lxd init --auto
sudo /snap/bin/lxc network create testbr0
sudo /snap/bin/lxc network attach-profile testbr0 default eth0
