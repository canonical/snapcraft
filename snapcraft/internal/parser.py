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

Parse a wiki index of parts and output formatted YAML for use by 'snapcraft'.
'snapcraft' uses the YAML to find non-local parts referred to by a project's
'snapcraft.yaml' file.  See 'snapcraft update' and 'snapcraft define' for more
details.


Usage:
  snapcraft-parser [options]

Options:
  -h --help                             show this help message and exit
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces)
  -i --index=<filepath>                 a file containing a part index.
                                        ({default_index!r})
  -o --output=<filename>                where to write the parsed parts list.
                                        ({default_parts_file!r})
"""

import logging
import os
import re
import urllib
import yaml
from yaml.scanner import ScannerError
from xdg import BaseDirectory
from yaml.parser import ParserError

from docopt import docopt
from collections import OrderedDict

import snapcraft
from snapcraft import project
from snapcraft.internal import errors, log, project_loader, repo, sources

logger = logging.getLogger(__name__)

# TODO: make this a temporary directory that gets removed when finished
BASE_DIR = os.path.join(BaseDirectory.xdg_cache_home, "snapcraft-parser")
PARTS_FILE = "snap-parts.yaml"
DEFAULT_INDEX = "http://wiki.ubuntu.com/snapcraft/parts?action=raw"

__doc__ = __doc__.format(default_index=DEFAULT_INDEX, default_parts_file=PARTS_FILE)


def _get_base_dir():
    """The location to which sources are downloaded"""
    return BASE_DIR


def main(argv=None):
    args = docopt(__doc__, version=snapcraft.__version__, argv=argv)

    # Default log level is INFO unless --debug is specified
    log_level = logging.INFO
    if args["--debug"]:
        log_level = logging.DEBUG

    log.configure(log_level=log_level)

    return run(args)


def _get_origin_data(origin_dir):
    origin_data = {}

    yaml_file = project.get_snapcraft_yaml(base_dir=origin_dir)
    try:
        with open(yaml_file) as fp:
            origin_data = yaml.safe_load(fp)
    except ScannerError as e:
        raise errors.InvalidWikiEntryError(e) from e

    return origin_data


def _is_local(source):
    # XXX: Is this sufficient?  Can ":" be part of a local
    # directory?
    return source is not None and ":" not in source


def _update_source(part, origin):
    # handle entry_parts with local sources
    orig_source = part.get("source")
    if orig_source == ".":
        part["source"] = origin
    elif _is_local(orig_source):
        part["source"] = origin
        part["source-subdir"] = orig_source

    return part


def _process_entry_parts(
    entry_parts, parts, origin, maintainer, description, origin_name, origin_version
):
    after_parts = set()
    parts_list = {}
    for part_name in entry_parts:
        if "/" in part_name:
            logger.warning(
                'DEPRECATED: Found a "/" in the name of the {!r} part'.format(part_name)
            )
        source_part = parts.get(part_name)
        replacements = project_loader.environment_to_replacements(
            {
                "SNAPCRAFT_PROJECT_NAME": origin_name,
                "SNAPCRAFT_PROJECT_VERSION": origin_version,
            }
        )

        source_part = project_loader.replace_attr(source_part, replacements)

        if source_part:
            source_part = _update_source(source_part, origin)
            source_part["maintainer"] = maintainer
            source_part["description"] = description

            parts_list[part_name] = source_part
            after = source_part.get("after", [])

            if after:
                after_parts.update(set(after))
        else:
            after_parts.add(part_name)

    return parts_list, after_parts


def _encode_origin(origin):
    return re.sub("[^A-Za-z0-9-_.]", "", origin)


def _process_entry(data):
    parts_list = OrderedDict()
    # Store all the parts listed in 'after' for each included part so that
    # we can check later that we aren't missing any parts.
    # XXX: What do we do about 'after' parts that should be looked for in
    # the wiki?  They should be in the master parts list.
    after_parts = set()

    # Get optional wiki entry fields.
    origin_type = data.get("origin-type")
    origin_branch = data.get("origin-branch")
    origin_commit = data.get("origin-commit")
    origin_tag = data.get("origin-tag")

    # Get required wiki entry fields.
    try:
        entry_parts = data["parts"]
        origin = data["origin"]
        maintainer = data["maintainer"]
        description = data["description"]
    except KeyError as e:
        raise errors.InvalidWikiEntryError("Missing key in wiki entry: {}".format(e))

    logger.info("Processing origin {origin!r}".format(origin=origin))
    origin_dir = os.path.join(_get_base_dir(), _encode_origin(origin))
    os.makedirs(origin_dir, exist_ok=True)

    source_handler = sources.get_source_handler(origin, source_type=origin_type)
    handler = source_handler(origin, source_dir=origin_dir)
    repo.check_for_command(handler.command)
    handler.source_branch = origin_branch
    handler.source_commit = origin_commit
    handler.source_tag = origin_tag
    handler.pull()

    try:
        origin_data = _get_origin_data(origin_dir)
    except project.errors.MissingSnapcraftYamlError as e:
        raise errors.InvalidWikiEntryError(
            "Origin {origin!r} is missing a snapcraft.yaml file.".format(origin=origin)
        ) from e
    except errors.SnapcraftEnvironmentError as e:
        raise errors.InvalidWikiEntryError("snapcraft.yaml error: {}".format(e)) from e

    origin_parts = origin_data.get("parts", {})
    origin_name = origin_data.get("name")
    origin_version = origin_data.get("version")

    entry_parts_list, entry_parts_after_list = _process_entry_parts(
        entry_parts,
        origin_parts,
        origin,
        maintainer,
        description,
        origin_name,
        origin_version,
    )
    parts_list.update(entry_parts_list)
    after_parts.update(entry_parts_after_list)

    return parts_list, after_parts


def _process_wiki_entry(
    entry, master_parts_list, master_missing_parts, pending_validation_entries
):
    """Add valid wiki entries to the master parts list"""
    # return the number of errors encountered
    try:
        data = yaml.safe_load(entry)
    except (ScannerError, ParserError) as e:
        raise errors.InvalidWikiEntryError(
            "Bad wiki entry, possibly malformed YAML for entry: {}".format(e)
        )

    try:
        parts = data["parts"]
    except KeyError as e:
        raise errors.InvalidWikiEntryError(
            '"parts" missing from wiki entry: {}'.format(entry)
        )
    for part_name in parts:
        if part_name and part_name in master_parts_list:
            raise errors.InvalidWikiEntryError(
                "Duplicate part found in the wiki: {} in entry {}".format(
                    part_name, entry
                )
            )

    parts_list, after_parts = _process_entry(data)

    known_parts = list(parts_list.keys()) + list(master_parts_list.keys())
    missing_parts = missing_parts_set(after_parts, known_parts)

    if not len(missing_parts):
        master_parts_list.update(parts_list)
        master_missing_parts -= set(parts_list.keys())
    else:
        pending_validation_entries.append(entry)
        master_missing_parts.update(missing_parts)
        logger.debug("Parts {!r} are missing".format(",".join(missing_parts)))


def _try_process_entry(
    entry, master_parts_list, missing_parts, pending_validation_entries
):
    wiki_errors = 0

    try:
        _process_wiki_entry(
            entry, master_parts_list, missing_parts, pending_validation_entries
        )
    except errors.SnapcraftError as e:
        logger.warning(e)
        wiki_errors += 1

    return wiki_errors


def _process_index(output):
    # XXX: This can't remain in memory if the list gets very large, but it
    # should be okay for now.
    master_parts_list = OrderedDict()
    pending_validation_entries = []
    missing_parts = set()

    wiki_errors = 0

    output = output.replace(b"{{{", b"").replace(b"}}}", b"")
    output = output.strip()

    # split the wiki into an array of entries to allow the parser to
    # proceed when invalid yaml is found.
    entry = ""
    for line in output.decode().splitlines():
        if line == "---":
            if entry:
                wiki_errors += _try_process_entry(
                    entry, master_parts_list, missing_parts, pending_validation_entries
                )
                entry = ""
        else:
            entry = "\n".join([entry, line])

    if entry:
        wiki_errors += _try_process_entry(
            entry, master_parts_list, missing_parts, pending_validation_entries
        )

    for entry in pending_validation_entries:
        wiki_errors += _try_process_entry(entry, master_parts_list, missing_parts, [])

    if len(missing_parts):
        logger.warning(
            "Parts {!r} are not defined in the parts entry".format(
                ",".join(missing_parts)
            )
        )
        wiki_errors += 1

    return {"master_parts_list": master_parts_list, "wiki_errors": wiki_errors}


def run(args):
    path = args.get("--output")
    if path is None:
        path = PARTS_FILE

    index = args.get("--index")
    if not index:
        index = DEFAULT_INDEX
    if "://" not in index:
        index = "{}{}".format("file://", os.path.join(os.getcwd(), index))
    try:
        output = urllib.request.urlopen(index).read()
    except urllib.error.URLError as e:
        logging.error("Unable to access index: {!r}".format(e))
        return 1

    data = _process_index(output)
    master_parts_list = data["master_parts_list"]
    wiki_errors = data["wiki_errors"]

    if wiki_errors:
        logger.warning("{} wiki errors found!".format(wiki_errors))

    _write_parts_list(path, master_parts_list)

    if args["--debug"]:
        print(yaml.dump(master_parts_list, default_flow_style=False))

    return wiki_errors


def missing_parts_set(parts, known_parts):
    missing_parts = set()
    for partname in parts:
        if partname not in known_parts:
            missing_parts.add(partname)

    return missing_parts


def _write_parts_list(path, master_parts_list):
    logger.info("Writing parts list to {!r}".format(path))
    with open(path, "w") as fp:
        fp.write(yaml.dump(master_parts_list, default_flow_style=False))
