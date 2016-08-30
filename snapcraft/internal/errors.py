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


class SnapcraftError(Exception):
    """Base class for all snapcraft exceptions.

    :cvar fmt: A format string that daughter classes override

    """
    fmt = 'Daughter classes should redefine this'

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __str__(self):
        return self.fmt.format([], **self.__dict__)


class PluginError(Exception):
    pass


class MissingState(Exception):
    pass


class SnapcraftPartConflictError(SnapcraftError):

    fmt = (
        'Parts {other_part_name!r} and {part_name!r} have the following file '
        'paths in common which have different contents:\n'
        '{file_paths}\n\n'
        'Snapcraft offers some capabilities to solve this by use of the '
        'following keywords:\n'
        '    - `filesets`\n'
        '    - `stage`\n'
        '    - `snap`\n'
        '    - `organize`\n\n'
        'Learn more about these part keywords by running '
        '`snapcraft help plugins`'
    )

    def __init__(self, *, part_name, other_part_name, conflict_files):
        spaced_conflict_files = ('    {}'.format(i) for i in conflict_files)
        super().__init__(part_name=part_name,
                         other_part_name=other_part_name,
                         file_paths='\n'.join(sorted(spaced_conflict_files)))
