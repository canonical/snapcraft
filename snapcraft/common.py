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

from snapcraft.internal.common import get_python2_path  # noqa
from snapcraft.internal.common import isurl  # noqa
from snapcraft.internal.common import get_include_paths  # noqa
from snapcraft.internal.common import get_library_paths  # noqa

# These are now available via file_utils, but don't break API.
from snapcraft.file_utils import link_or_copy  # noqa
from snapcraft.file_utils import replace_in_file  # noqa

# These are now available via formatting_utils, but don't break API.
from snapcraft.formatting_utils import combine_paths  # noqa
from snapcraft.formatting_utils import format_path_variable  # noqa
