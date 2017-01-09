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

import difflib
import logging
import os
import sys

import requests
import yaml
from xdg import BaseDirectory

from snapcraft.internal.indicators import download_requests_stream
from snapcraft.internal.common import get_terminal_width
from snapcraft.internal.errors import SnapcraftPartMissingError
from snapcraft.internal import (
    deprecations,
    pluginhandler,
    project_loader,
    repo
)


PARTS_URI = 'https://parts.snapcraft.io/v1/parts.yaml'
_MATCH_RATIO = 0.6
_HEADER_PART_NAME = 'PART NAME'
_HEADER_DESCRIPTION = 'DESCRIPTION'

logging.getLogger("urllib3").setLevel(logging.CRITICAL)
logger = logging.getLogger(__name__)


class _Base:

    def __init__(self):
        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        os.makedirs(self.parts_dir, exist_ok=True)
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')


class _Update(_Base):

    def __init__(self):
        super().__init__()
        self._headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')
        self._parts_uri = os.environ.get('SNAPCRAFT_PARTS_URI', PARTS_URI)

    def execute(self):
        headers = self._load_headers()
        self._request = requests.get(self._parts_uri, stream=True,
                                     headers=headers)

        if self._request.status_code == 304:
            logger.info('The parts cache is already up to date.')
            return
        self._request.raise_for_status()

        download_requests_stream(self._request, self.parts_yaml,
                                 'Downloading parts list')
        self._save_headers()

    def _load_headers(self):
        if not os.path.exists(self._headers_yaml):
            return None

        with open(self._headers_yaml) as headers_file:
            return yaml.load(headers_file)

    def _save_headers(self):
        headers = {
            'If-Modified-Since': self._request.headers.get('Last-Modified')}

        with open(self._headers_yaml, 'w') as headers_file:
            headers_file.write(yaml.dump(headers))


class _RemoteParts(_Base):

    def __init__(self):
        super().__init__()

        if not os.path.exists(self.parts_yaml):
            update()

        with open(self.parts_yaml) as parts_file:
            self._parts = yaml.load(parts_file)

    def get_part(self, part_name, full=False):
        try:
            remote_part = self._parts[part_name].copy()
        except KeyError:
            raise SnapcraftPartMissingError(part_name=part_name)
        if not full:
            for key in ['description', 'maintainer']:
                remote_part.pop(key)
        return remote_part

    def matches_for(self, part_match, max_len=0):
        matcher = difflib.SequenceMatcher(isjunk=None, autojunk=False)
        matcher.set_seq2(part_match)

        matching_parts = {}
        for part_name in self._parts.keys():
            matcher.set_seq1(part_name)
            add_part_name = matcher.ratio() >= _MATCH_RATIO

            if add_part_name or (part_match in part_name):
                matching_parts[part_name] = self._parts[part_name]
                if len(part_name) > max_len:
                    max_len = len(part_name)

        return matching_parts, max_len

    def compose(self, part_name, properties):
        """Return properties composed with the ones from part name in the wiki.
        :param str part_name: The name of the part to query from the wiki
        :param dict properties: The current set of properties
        :return: Part properties from the wiki composed with the properties
                 passed as a parameter.
        :rtype: dict
        :raises KeyError: if part_name is not found in the wiki.
        """
        remote_part = self.get_part(part_name)
        remote_part.update(properties)

        return remote_part


class SnapcraftLogicError(Exception):

    @property
    def message(self):
        return self._message

    def __init__(self, message):
        self._message = message


class PartsConfig:

    def __init__(self, parts, project_options, validator, build_tools,
                 snapcraft_yaml):
        self._snap_name = parts['name']
        self._confinement = parts['confinement']
        self._parts_data = parts.get('parts', {})
        self._project_options = project_options
        self._validator = validator
        self.build_tools = build_tools
        self._snapcraft_yaml = snapcraft_yaml

        self.all_parts = []
        self._part_names = []
        self.after_requests = {}

        self._process_parts()

    @property
    def part_names(self):
        return self._part_names

    def _process_parts(self):
        for part_name in self._parts_data:
            if '/' in part_name:
                logger.warning('DEPRECATED: Found a "/" '
                               'in the name of the {!r} part'.format(
                                part_name))
            self._part_names.append(part_name)
            properties = self._parts_data[part_name] or {}

            plugin_name = properties.get('plugin')

            if 'after' in properties:
                self.after_requests[part_name] = properties.pop('after')

            if 'filesets' in properties:
                del properties['filesets']

            # FIXME: snap is deprecated, rewrite it to prime instead.
            if properties.get('snap'):
                deprecations.handle_deprecation_notice('dn1')
                properties['prime'] = properties.pop('snap')

            self.load_plugin(part_name, plugin_name, properties)

        self._compute_dependencies()
        self.all_parts = self._sort_parts()

    def _compute_dependencies(self):
        '''Gather the lists of dependencies and adds to all_parts.'''

        for part in self.all_parts:
            dep_names = self.after_requests.get(part.name, [])
            for dep in dep_names:
                for i in range(len(self.all_parts)):
                    if dep == self.all_parts[i].name:
                        part.deps.append(self.all_parts[i])
                        break

    def _sort_parts(self):
        '''Performs an inneficient but easy to follow sorting of parts.'''
        sorted_parts = []

        while self.all_parts:
            top_part = None
            for part in self.all_parts:
                mentioned = False
                for other in self.all_parts:
                    if part in other.deps:
                        mentioned = True
                        break
                if not mentioned:
                    top_part = part
                    break
            if not top_part:
                raise SnapcraftLogicError(
                    'circular dependency chain found in parts definition')
            sorted_parts = [top_part] + sorted_parts
            self.all_parts.remove(top_part)

        return sorted_parts

    def get_prereqs(self, part_name):
        """Returns a set with all of part_names' prerequisites."""
        return set(self.after_requests.get(part_name, []))

    def get_dependents(self, part_name):
        """Returns a set of all the parts that depend upon part_name."""

        dependents = set()
        for part, prerequisites in self.after_requests.items():
            if part_name in prerequisites:
                dependents.add(part)

        return dependents

    def get_part(self, part_name):
        for part in self.all_parts:
            if part.name == part_name:
                return part

        return None

    def clean_part(self, part_name, staged_state, primed_state, step):
        part = self.get_part(part_name)
        part.clean(staged_state, primed_state, step)

    def validate(self, part_names):
        for part_name in part_names:
            if part_name not in self._part_names:
                raise EnvironmentError(
                    'The part named {!r} is not defined in '
                    '{!r}'.format(part_name, self._snapcraft_yaml))

    def load_plugin(self, part_name, plugin_name, part_properties):
        part = pluginhandler.load_plugin(
            part_name,
            plugin_name=plugin_name,
            part_properties=part_properties,
            project_options=self._project_options,
            part_schema=self._validator.part_schema)

        self.build_tools += part.code.build_packages
        if part.source_handler and part.source_handler.command:
            self.build_tools.append(
                repo.get_packages_for_source_type(part.source_handler.command))
        self.all_parts.append(part)

        return part

    def build_env_for_part(self, part, root_part=True):
        """Return a build env of all the part's dependencies."""

        env = []
        stagedir = self._project_options.stage_dir
        core_dynamic_linker = self._project_options.get_core_dynamic_linker()

        if root_part:
            # this has to come before any {}/usr/bin
            env += part.env(part.installdir)
            env += project_loader._runtime_env(
                part.installdir, self._project_options.arch_triplet)
            env += project_loader._runtime_env(
                stagedir, self._project_options.arch_triplet)
            env += project_loader._build_env(
                part.installdir,
                self._snap_name,
                self._confinement,
                self._project_options.arch_triplet,
                core_dynamic_linker=core_dynamic_linker)
            env += project_loader._build_env_for_stage(
                stagedir,
                self._snap_name,
                self._confinement,
                self._project_options.arch_triplet,
                core_dynamic_linker=core_dynamic_linker)
            env.append('SNAPCRAFT_PART_INSTALL={}'.format(part.installdir))
        else:
            env += part.env(stagedir)
            env += project_loader._runtime_env(
                stagedir, self._project_options.arch_triplet)

        for dep_part in part.deps:
            env += dep_part.env(stagedir)
            env += self.build_env_for_part(dep_part, root_part=False)

        return env


def update():
    _Update().execute()


def define(part_name):
    try:
        remote_part = _RemoteParts().get_part(part_name, full=True)
    except SnapcraftPartMissingError as e:
        raise RuntimeError(
            'Cannot find the part name {!r} in the cache. Please '
            'run `snapcraft update` and try again.\nIf it is indeed missing, '
            'consider going to https://wiki.ubuntu.com/snapcraft/parts '
            'to add it.'.format(part_name)) from e
    print('Maintainer: {!r}'.format(remote_part.pop('maintainer')))
    print('Description: {}'.format(remote_part.pop('description')))
    print('')
    yaml.dump({part_name: remote_part},
              default_flow_style=False, stream=sys.stdout)


def search(part_match):
    header_len = len(_HEADER_PART_NAME)
    matches, part_length = _RemoteParts().matches_for(part_match, header_len)

    terminal_width = get_terminal_width(max_width=None)
    part_length = max(part_length, header_len)
    # <space> + <space> + <description> + ... = 5
    description_space = terminal_width - part_length - 5

    if not matches:
        # apt search does not return error, we probably shouldn't either.
        logger.info('No matches found, try to run `snapcraft update` to '
                    'refresh the remote parts cache.')
        return

    print('{}  {}'.format(
        _HEADER_PART_NAME.ljust(part_length, ' '), _HEADER_DESCRIPTION))
    for part_key in sorted(matches.keys()):
        description = matches[part_key]['description'].split('\n')[0]
        if len(description) > description_space:
            description = '{}...'.format(description[0:description_space])
        print('{}  {}'.format(
            part_key.ljust(part_length, ' '), description))


def get_remote_parts():
    return _RemoteParts()
