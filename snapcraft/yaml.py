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

import contextlib
import jsonschema
import logging
import os
import os.path
import yaml

import snapcraft.lifecycle
import snapcraft.wiki
from snapcraft import common


logger = logging.getLogger(__name__)


_DEPRECATION_LIST = [
    'ant-project',
    'autotools-project',
    'cmake-project',
    'go-project',
    'make-project',
    'maven-project',
    'python2-project',
    'python3-project',
]


@jsonschema.FormatChecker.cls_checks('file-path')
@jsonschema.FormatChecker.cls_checks('icon-path')
def _validate_file_exists(instance):
    return os.path.exists(instance)


class SnapcraftYamlFileError(Exception):

    @property
    def file(self):
        return self._file

    def __init__(self, yaml_file):
        self._file = yaml_file


class SnapcraftLogicError(Exception):

    @property
    def message(self):
        return self._message

    def __init__(self, message):
        self._message = message


class SnapcraftSchemaError(Exception):

    @property
    def message(self):
        return self._message

    def __init__(self, message):
        self._message = message


class PluginNotDefinedError(Exception):

    @property
    def part(self):
        return self._part

    def __init__(self, part):
        self._part = part


class Config:

    def __init__(self):
        self.build_tools = []
        self.all_parts = []
        after_requests = {}

        self.data = _snapcraft_yaml_load()
        _validate_snapcraft_yaml(self.data)

        self.build_tools = self.data.get('build-packages', [])

        self._wiki = snapcraft.wiki.Wiki()

        for part_name in self.data.get("parts", []):
            properties = self.data["parts"][part_name] or {}

            plugin_name = properties.pop('plugin', None)
            # TODO search the wiki
            if not plugin_name and 'type' in properties:
                plugin_name = properties.pop('type')
                logger.warning('DEPRECATED: Use "plugin" instead of "type"')
            elif not plugin_name:
                logger.info(
                    'Searching the wiki to compose part "{}"'.format(
                        part_name))
                with contextlib.suppress(KeyError):
                    properties = self._wiki.compose(part_name, properties)
                    plugin_name = properties.pop('plugin', None)

            if not plugin_name:
                raise PluginNotDefinedError(part_name)

            if plugin_name in _DEPRECATION_LIST:
                plugin_name = plugin_name.rsplit('-project')[0]
                logger.warning(
                    'DEPRECATED: plugin names ending in -project are '
                    'deprecated. Using {0} instead of {0}-project'.format(
                        plugin_name))

            if "after" in properties:
                after_requests[part_name] = properties.pop('after')

            properties['stage'] = _expand_filesets_for('stage', properties)
            properties['snap'] = _expand_filesets_for('snap', properties)

            if 'filesets' in properties:
                del properties['filesets']

            self.load_plugin(part_name, plugin_name, properties)

        self._compute_part_dependencies(after_requests)
        self.all_parts = self._sort_parts()

    def _compute_part_dependencies(self, after_requests):
        '''Gather the lists of dependencies and adds to all_parts.'''

        for part in self.all_parts:
            dep_names = after_requests.get(part.name, [])
            for dep in dep_names:
                found = False
                for i in range(len(self.all_parts)):
                    if dep in self.all_parts[i].name:
                        part.deps.append(self.all_parts[i])
                        found = True
                        break
                if not found:
                    wiki_part = self._wiki.get_part(dep)
                    found = True if wiki_part else False
                    if found:
                        plugin_name = wiki_part.pop('plugin')
                        part.deps.append(self.load_plugin(
                            dep, plugin_name, wiki_part))
                if not found:
                    raise SnapcraftLogicError(
                        'part name missing {}'.format(dep))

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

    def load_plugin(self, part_name, plugin_name, properties):
        part = snapcraft.lifecycle.load_plugin(
            part_name, plugin_name, properties)

        self.build_tools += part.code.build_packages
        self.all_parts.append(part)
        return part

    def runtime_env(self, root):
        env = []
        env.append('PATH="' + ':'.join([
            '{0}/bin',
            '{0}/usr/bin',
            '$PATH'
        ]).format(root) + '"')
        env.append('LD_LIBRARY_PATH="' + ':'.join([
            '{0}/lib',
            '{0}/usr/lib',
            '{0}/lib/{1}',
            '{0}/usr/lib/{1}',
            '$LD_LIBRARY_PATH'
        ]).format(root, snapcraft.common.get_arch_triplet()) + '"')
        return env

    def build_env(self, root):
        env = []
        env.append('CFLAGS="' + ' '.join([
            '-I{0}/include',
            '-I{0}/usr/include',
            '-I{0}/include/{1}',
            '-I{0}/usr/include/{1}',
            '$CFLAGS'
        ]).format(root, snapcraft.common.get_arch_triplet()) + '"')
        env.append('CPPFLAGS="' + ' '.join([
            '-I{0}/include',
            '-I{0}/usr/include',
            '-I{0}/include/{1}',
            '-I{0}/usr/include/{1}',
            '$CPPFLAGS'
        ]).format(root, snapcraft.common.get_arch_triplet()) + '"')
        env.append('LDFLAGS="' + ' '.join([
            '-L{0}/lib',
            '-L{0}/usr/lib',
            '-L{0}/lib/{1}',
            '-L{0}/usr/lib/{1}',
            '$LDFLAGS'
        ]).format(root, snapcraft.common.get_arch_triplet()) + '"')
        env.append('PKG_CONFIG_SYSROOT_DIR={0}'.format(root))
        env.append('PKG_CONFIG_PATH=' + ':'.join([
            '{0}/usr/lib/pkgconfig',
            '{0}/usr/lib/{1}/pkgconfig',
            '{0}/usr/share/pkgconfig',
            '{0}/usr/local/lib/pkgconfig',
            '{0}/usr/local/lib/{1}/pkgconfig',
            '{0}/usr/local/share/pkgconfig',
            '$PKG_CONFIG_PATH'
        ]).format(root, snapcraft.common.get_arch_triplet()))
        env.append('PERL5LIB={0}/usr/share/perl5/'.format(root))
        return env

    def build_env_for_part(self, part):
        # Grab build env of all part's dependencies

        env = []

        for dep in part.deps:
            root = dep.installdir
            env += dep.env(root)
            env += self.build_env_for_part(dep)

        env += part.env(part.installdir)
        env += self.runtime_env(part.installdir)
        env += self.build_env(part.installdir)

        return env

    def stage_env(self):
        root = common.get_stagedir()
        env = []

        env += self.runtime_env(root)
        env += self.build_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env

    def snap_env(self):
        root = common.get_snapdir()
        env = []

        env += self.runtime_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env


def _validate_snapcraft_yaml(snapcraft_yaml):
    schema_file = os.path.abspath(os.path.join(common.get_schemadir(),
                                               'snapcraft.yaml'))

    try:
        with open(schema_file) as fp:
            schema = yaml.load(fp)
            format_check = jsonschema.FormatChecker()
            jsonschema.validate(snapcraft_yaml, schema,
                                format_checker=format_check)
    except FileNotFoundError:
        raise SnapcraftSchemaError(
            'snapcraft validation file is missing from installation path')
    except jsonschema.ValidationError as e:
        raise SnapcraftSchemaError(e.message)


def _snapcraft_yaml_load(yaml_file='snapcraft.yaml'):
    try:
        with open(yaml_file) as fp:
            return yaml.load(fp)
    except FileNotFoundError:
        raise SnapcraftYamlFileError(yaml_file)
    except yaml.scanner.ScannerError as e:
        raise SnapcraftSchemaError(
            '{} on line {} of {}'.format(
                e.problem, e.problem_mark.line, yaml_file))


def _expand_filesets_for(stage, properties):
    filesets = properties.get('filesets', {})
    fileset_for_stage = properties.get(stage, {})
    new_stage_set = []

    for item in fileset_for_stage:
        if item.startswith('$'):
            try:
                new_stage_set.extend(filesets[item[1:]])
            except KeyError:
                raise SnapcraftLogicError(
                    '\'{}\' referred to in the \'{}\' fileset but it is not '
                    'in filesets'.format(item, stage))
        else:
            new_stage_set.append(item)

    return new_stage_set
