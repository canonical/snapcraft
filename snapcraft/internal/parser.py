#!/usr/bin/python3
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

"""
snapcraft-parser

Usage:
  snapcraft-parser [options]

Options:
  -h --help                             show this help message and exit
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces)
  -i --index=<filename>                 a file containing a part index.
  -o --output=<filename>                where to write the parsed parts list.
"""

import logging
import os
import pkg_resources
import sys
import textwrap
import urllib
import yaml

from docopt import docopt

from snapcraft.internal import log, sources


logger = logging.getLogger(__name__)


PART_NAMESPACE_SEP = '/'
# TODO: make this a temporary directory that get's removed when finished
BASE_DIR = "/tmp"
PARTS_FILE = "snap-parts.yaml"


def _get_version():
    try:
        return pkg_resources.require('snapcraft-parser')[0].version
    except pkg_resources.DistributionNotFound:
        return 'devel'


def main(argv=None):
    args = docopt(__doc__, version=_get_version(), argv=argv)

    # Default log level is INFO unless --debug is specified
    log_level = logging.INFO
    if args['--debug']:
        log_level = logging.DEBUG

    log.configure(log_level=log_level)

    try:
        return run(args)
    except Exception as e:
        if args['--debug']:
            raise

        sys.exit(textwrap.fill(str(e)))


def _get_namespaced_partname(key, partname):
    return "{key}{sep}{partname}".format(
        key=key,
        sep=PART_NAMESPACE_SEP,
        partname=partname,
    )


def _update_after_parts(partname, after_parts):
    return [_get_namespaced_partname(partname, x) for x in after_parts]


def _get_origin_data(origin_dir):
    origin_data = {}
    with open(os.path.join(origin_dir, 'snapcraft.yaml'), "r") as fp:
        origin_data = yaml.load(fp)

    return origin_data


def _is_local(source):
    # XXX: Is this sufficient?  Can ":" be part of a local
    # directory?
    return ':' not in source


def _update_source(part, origin):
    # handle subparts with local sources
    orig_source = part.get("source")
    if orig_source == ".":
        part["source"] = origin
    elif _is_local(orig_source):
        part["source"] = origin
        part["source-subdir"] = orig_source

    return part


def _process_subparts(project_part, subparts, parts, origin, maintainer,
                      description):
    after_parts = set()
    parts_list = {}
    for part in subparts:
        source_part = parts.get(part)
        # TODO: get any dependent parts here as well using
        # 'project_part' to namespace them.
        if source_part:
            source_part = _update_source(source_part, origin)
            source_part['maintainer'] = maintainer
            source_part['description'] = description

            namespaced_part = _get_namespaced_partname(project_part, part)
            parts_list[namespaced_part] = source_part
            after = source_part.get("after", [])

            if after:
                after = _update_after_parts(project_part, after)
                after_parts.update(set(after))

    return parts_list, after_parts


def _process_index(output):
    # XXX: This can't remain in memory if the list gets very large, but it
    # should be okay for now.
    master_parts_list = {}

    output = output.strip()

    output = output.replace(b"{{{", b"").replace(b"}}}", b"")

    all_data = yaml.load_all(output)
    for data in all_data:
        key = data.get('project-part')
        parts_list = {}
        # Store all the parts listed in 'after' for each included part so that
        # we can check later that we aren't missing any parts.
        # XXX: What do we do about 'after' parts that should be looked for in
        # the wiki?  They should be in the master parts list.
        after_parts = set()

        logger.debug('Processing part {!r}'.format(key))

        try:
            origin = data['origin']
            origin_type = data.get('origin-type')
            project_part = data['project-part']
            subparts = data['parts']
            maintainer = data['maintainer']
            description = data['description']
        except KeyError as e:
            logger.warning("Missing key in wiki entry: {}".format(e))
            continue  # next entry

        # TODO: this should really be based on the origin uri not
        # the part name to avoid the situation where there are multiple
        # parts being pulled from the same repo branch.
        origin_dir = os.path.join(BASE_DIR, key)
        try:
            os.makedirs(origin_dir)
        except FileExistsError:
            pass

        class Options():
            pass

        options = Options()
        setattr(options, 'source', origin)

        if origin_type:
            setattr(options, 'source_type', origin_type)

        sources.get(origin_dir, None, options)

        origin_data = _get_origin_data(origin_dir)

        parts = origin_data.get('parts', {})

        source_part = parts.get(project_part)
        if source_part:
            source_part = _update_source(source_part, origin)
            source_part['maintainer'] = maintainer
            source_part['description'] = description
            after = source_part.get('after', [])

            if after:
                after = _update_after_parts(project_part, after)
                after_parts.update(set(after))

            parts_list[project_part] = source_part
            subparts_list, subparts_after_list = _process_subparts(
                project_part, subparts, parts, origin, maintainer,
                description,
            )
            parts_list.update(subparts_list)
            after_parts.update(subparts_after_list)

        if is_valid_parts_list(parts_list, after_parts):
            master_parts_list.update(parts_list)

    return master_parts_list


def run(args):
    path = args.get('--output')
    if path is None:
        path = PARTS_FILE

    index = args.get('--index')
    if index:
        if '://' not in index:
            index = '{}{}'.format(
                'file://', os.path.join(os.getcwd(), index))
        output = urllib.request.urlopen(index).read()
    else:
        # XXX: fetch the index from the wiki
        output = '{}'

    master_parts_list = _process_index(output)
    _write_parts_list(path, master_parts_list)

    if args['--debug']:
        print(yaml.dump(master_parts_list, default_flow_style=False))

    return args


def is_valid_parts_list(parts_list, parts):
    for partname in parts:
        if partname not in parts_list.keys():
            logging.error('Part {!r} is missing from the parts entry'.format(
                partname))
            return False

    return True


def _write_parts_list(path, master_parts_list):
    logging.debug('Writing parts list to {!r}'.format(path))
    with open(path, 'w') as fp:
        fp.write(yaml.dump(master_parts_list,
                 default_flow_style=False))


if __name__ == '__main__':  # pragma: no cover
    main()                  # pragma: no cover
