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
  -p --print                            print the master parts list
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces)
  -i --index=<index>                    a file containing a part index.
"""

import gzip
import logging
import os
import pkg_resources
import sys
import textwrap
import urllib
import yaml

from docopt import docopt

import snapcraft_parser
from snapcraft.internal import log, sources


logger = logging.getLogger(__name__)


PART_NAMESPACE_SEP = '/'
# TODO: make this a temporary directory that get's removed when finished
BASE_DIR = "/tmp"
PARTS_FILE = os.path.join(BASE_DIR, "snap-parts.yaml.gz")


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


def _update_after_parts(partname, after_parts):
    return [_namespaced_partname(partname, x) for x in after_parts]


def _get_origin_data(origin_dir):
    origin_data = {}
    with open(os.path.join(origin_dir, 'snapcraft.yaml'), "r") as fp:
        origin_data = yaml.load(fp)

    return origin_data


def run(args):
    index = args.get('--index')
    if index is not None:
        if "://" not in index:
            index = "%s%s" % ("file://", os.path.join(os.getcwd(), index))
        output = urllib.request.urlopen(index).read()
    else:
        output = "{}"

    # XXX: This can't remain in memory if the list gets very large, but it
    # should be okay for now.
    master_parts_list = {}

    data = yaml.load(output)
    for key, value in data.items():
        parts_list = {}
        # Store all the parts listed in 'after' for each included part so that
        # we can check later that we aren't missing any parts.
        # XXX: What do we do about 'after' parts that should be looked for in
        # the wiki?  They should be in the master parts list.
        after_parts = set()

        logger.debug("Processing part %s", key)
        origin = data[key].get("origin")
        origin_type = data[key].get("origin-type")
        project_part = data[key].get("project-part")
        subparts = data[key].get("parts", [])

        if origin is not None:
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
            setattr(options, "source", origin)

            if origin_type is not None:
                setattr(options, "source_type", origin_type)

            sources.get(origin_dir, None, options)

            origin_data = _get_origin_data(origin_dir)

            parts = origin_data.get('parts', {})

            source_part = parts.get(project_part)
            if source_part is not None:
                after = source_part.get('after', [])

                if after:
                    after = _update_after_parts(project_part, after)
                    after_parts.update(set(after))

                parts_list[project_part] = source_part

            for part in subparts:
                source_part = parts.get(part)
                # TODO: get any dependent parts here as well using
                # 'project_part' to namespace them.
                if source_part is not None:
                    parts_list[_namespaced_partname(project_part,
                                                    part)] = source_part
                    after = source_part.get("after", [])

                    if after:
                        after = _update_after_parts(project_part, after)
                        after_parts.update(set(after))

        if _valid_parts_list(parts_list, after_parts):
            master_parts_list.update(parts_list)

    _write_gzipped_parts_list(BASE_DIR, master_parts_list)


    if args['--print']:
        print(yaml.dump(master_parts_list, default_flow_style=False))

    return args

def _valid_parts_list(parts_list, parts):
    for partname in parts:
        if partname not in parts_list.keys():
            logging.error(
                "Part '%s' is referenced by another part but not included in the master parts list" % partname)
            return False

    return True

def _write_gzipped_parts_list(base_dir, master_parts_list):
    with gzip.open(PARTS_FILE, "w") as fgp:
        fgp.write(yaml.dump(master_parts_list,
                            default_flow_style=False).encode("utf-8"))


def _namespaced_partname(key, partname):
    return "{key}{sep}{partname}".format(
        key=key,
        sep=PART_NAMESPACE_SEP,
        partname=partname,
    )


if __name__ == '__main__':  # pragma: no cover
    main()                  # pragma: no cover
