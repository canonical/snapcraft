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

from snapcraft.internal.meta import (
    _appstream,
    _errors
)


def get_info(keys, config_data, parts_dir):
    if 'adopt-info' in config_data:
        part = config_data['adopt-info']
        if part in config_data['parts']:
            if 'parse-info' in config_data['parts'][part]:
                part_dir = os.path.join(parts_dir, part)
                info_paths = config_data['parts'][part]['parse-info']
                return _extract_info_from_part(
                    keys,
                    config_data['parts'][part]['plugin'],
                    part_dir,
                    info_paths)
            else:
                raise _errors.SnapcraftYamlMissingRequiredValue('parse-info')
        else:
            raise _errors.WrongAdoptInfo(part)
    else:
        raise _errors.SnapcraftYamlMissingRequiredValue('adopt-info')


def _extract_info_from_part(keys, plugin, part_dir, info_paths):
    info = {}
    for path in info_paths:
        if keys:
            source_full_paths = [
                os.path.join(part_dir, step_dir, path)
                for step_dir in ('install', 'build')]
            for full_info_path in source_full_paths:
                if os.path.exists(full_info_path):
                    path_info = _get_info_parser(
                        plugin, full_info_path).extract(keys)
                    for key in path_info:
                        keys.remove(key)
                    # Do not search the build dir if the file exists in the
                    # install dir.
                    break
            else:
                raise _errors.UnexistingSourceMetaPath(source_full_paths[0])
            info.update(path_info)
    if keys:
        raise _errors.MissingSnapcraftYamlKeys(keys=keys)
    else:
        return info


def _get_info_parser(plugin, path):
    if path.endswith('.metainfo.xml'):
        return _appstream.AppstreamInfoParser(path)
    else:
        raise _errors.SourceMetadataParserError(path=path)
