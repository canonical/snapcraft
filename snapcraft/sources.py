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

import sys as _sys

if _sys.platform == "linux":
    from snapcraft.internal.sources import Script  # noqa
    from snapcraft.internal.sources import Bazaar  # noqa
    from snapcraft.internal.sources import Git  # noqa
    from snapcraft.internal.sources import Mercurial  # noqa
    from snapcraft.internal.sources import Subversion  # noqa
    from snapcraft.internal.sources import Tar  # noqa
    from snapcraft.internal.sources import Local  # noqa
    from snapcraft.internal.sources import Zip  # noqa
    from snapcraft.internal.sources import get  # noqa
    from snapcraft.internal.sources import Deb  # noqa
    from snapcraft.internal.sources import Rpm  # noqa
