# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from snapcraft.internal.project_loader.grammar import _to


def package_transformer(call_stack, package_name, project_options):
    if any(isinstance(s, _to.ToStatement) for s in call_stack):
        if ':' not in package_name:
            # deb_arch is target arch or host arch if both are the same
            package_name += ':{}'.format(project_options.deb_arch)

    return package_name
